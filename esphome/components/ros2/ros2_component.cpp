#include "ros2_component.h"

#include <cmath>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "ros2_json.h"

namespace esphome
{
  namespace ros2
  {

    static const char *const TAG = "ros2";

    float Ros2Component::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

    void Ros2Component::setup()
    {
      this->try_subscribe_();
      this->register_camera_listener_();
    }

    void Ros2Component::register_camera_listener_()
    {
#ifdef USE_CAMERA
      bool want_images = false;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].kind == PubKind::IMAGE_SINGLE && this->pubs_[i].camera != nullptr)
        {
          want_images = true;
          break;
        }
      }
      if (want_images && camera::Camera::instance() != nullptr)
        camera::Camera::instance()->add_listener(this);
      else if (want_images)
        ESP_LOGE(TAG, "Image publication configured but no camera instance found");
#else
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].kind == PubKind::IMAGE_SINGLE)
          ESP_LOGE(TAG, "camera support not compiled in");
      }
#endif
    }

    void Ros2Component::on_camera_image(const std::shared_ptr<camera::CameraImage> &image)
    {
      if (this->mw_ == nullptr || !this->mw_->connected())
        return;
      if (image == nullptr || image->get_data_length() == 0)
        return;
      const uint32_t now = App.get_loop_component_start_time();
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].kind != PubKind::IMAGE_SINGLE)
          continue;
        if (now - this->pubs_[i].last_pub < this->pubs_[i].interval_ms)
          continue;
        this->pubs_[i].last_pub = now;
        this->publish_compressed_image_(this->pubs_[i], image->get_data_buffer(), image->get_data_length());
      }
    }

    void Ros2Component::publish_compressed_image_(Publication &pub, const uint8_t *jpeg, size_t len)
    {
      if (this->mw_ == nullptr)
        return;
      MiddlewareOptions opts;
      opts.reliable = pub.reliable;
      opts.qos_explicit = pub.qos_explicit;
      opts.frame_id = pub.frame_id;
      if (this->time_ != nullptr)
      {
        ESPTime t = this->time_->utcnow();
        if (t.is_valid())
        {
          opts.stamp_sec = (int32_t) t.timestamp;
          opts.stamp_nsec = 0;
        }
      }
      if (!this->mw_->publish_image(pub.topic, jpeg, len, &opts))
        ESP_LOGW(TAG, "Image publish failed for %s (%u bytes)", pub.topic.c_str(), (unsigned) len);
    }

    void Ros2Component::fill_header_(HeaderMsg &header, const char *frame_id)
    {
      if (this->time_ != nullptr)
      {
        ESPTime t = this->time_->utcnow();
        if (t.is_valid())
        {
          header.stamp_sec = (int32_t) t.timestamp;
          header.stamp_nsec = 0;
        }
      }
      if (frame_id != nullptr)
      {
        strncpy(header.frame_id, frame_id, ROS2_FRAME_ID_LEN - 1);
        header.frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
      }
    }

    void Ros2Component::apply_qos_(bool &reliable, bool &explicit_out, const char *qos)
    {
      if (qos == nullptr || *qos == '\0')
        return;
      explicit_out = true;
      reliable = strcmp(qos, "best_effort") != 0;
    }

    void Ros2Component::try_subscribe_()
    {
      if (this->subscribed_)
        return;
      if (this->mw_ == nullptr)
      {
        this->mw_ = MiddlewareRegistry::find(this->middleware_name_);
        if (this->mw_ == nullptr)
        {
          const uint32_t now = App.get_loop_component_start_time();
          if (now >= this->mw_retry_at_)
          {
            ESP_LOGW(TAG, "Middleware '%s' not ready yet; retrying", this->middleware_name_.c_str());
            this->mw_retry_at_ = now + 1000;
          }
          return;
        }
      }
      for (size_t i = 0; i < this->num_subs_; i++)
      {
        const Subscription &sub = this->subs_[i];
        if (sub.type == nullptr)
          continue;
        MiddlewareOptions opts;
        opts.reliable = sub.reliable;
        opts.qos_explicit = sub.qos_explicit;
        this->mw_->subscribe(
            sub.topic, sub.type,
            [this, i](const void *sample, size_t len)
            {
              (void)len;
              const Subscription &s = this->subs_[i];
              const void *p = sample;
              if (s.kind == SubKind::JOINT_MULTI)
              {
                this->dispatch_joints_(s, p);
              }
              else if (s.kind == SubKind::SWITCH_SINGLE)
              {
                this->dispatch_scalar_switch_(s, p);
              }
              else if (s.kind == SubKind::SERVO_SINGLE)
              {
                this->dispatch_scalar_servo_(s, p);
              }
              else if (s.kind == SubKind::LIGHT_SINGLE)
              {
                this->dispatch_light_(s, p);
              }
              else if (s.kind == SubKind::DIFF_DRIVE)
              {
                this->dispatch_diff_drive_(s, p);
              }
            },
            &opts);
        ESP_LOGI(TAG, "Subscribed: %s (%s)", sub.topic.c_str(), sub.type->name);
      }
      this->subscribed_ = true;
    }

    void Ros2Component::loop()
    {
      this->try_subscribe_();
      const uint32_t now = App.get_loop_component_start_time();
      this->stop_stale_diff_drive_(now);
      for (size_t i = 0; i < this->num_pubs_; i++)
        this->poll_publication_(this->pubs_[i]);
#ifdef USE_BINARY_SENSOR
      if (this->status_sensor_ != nullptr && this->mw_ != nullptr)
        this->status_sensor_->publish_state(this->mw_->connected());
#endif
    }

    void Ros2Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "ROS 2 bridge:");
      ESP_LOGCONFIG(TAG, "  Middleware: %s (%s)", this->middleware_name_.c_str(),
                    this->mw_ != nullptr ? "found" : "MISSING");
      ESP_LOGCONFIG(TAG, "  Subscriptions: %u, Publications: %u", (unsigned)this->num_subs_,
                    (unsigned)this->num_pubs_);
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].kind == PubKind::IMAGE_SINGLE)
          ESP_LOGCONFIG(TAG, "  Image: %s (CompressedImage)", this->pubs_[i].topic.c_str());
      }
    }

    void Ros2Component::dispatch_scalar_switch_(const Subscription &sub, const void *sample)
    {
#ifdef USE_SWITCH
      if (sub.sw == nullptr || sub.type == nullptr)
        return;
      if (strcmp(sub.type->name, "std_msgs/Bool") != 0)
        return;
      bool on = static_cast<const BoolMsg *>(sample)->data;
      if (on)
      {
        sub.sw->turn_on();
      }
      else
      {
        sub.sw->turn_off();
      }
#else
      (void)sub;
      (void)sample;
#endif
    }

    void Ros2Component::dispatch_scalar_servo_(const Subscription &sub, const void *sample)
    {
#ifdef USE_SERVO
      if (sub.servo == nullptr || sub.type == nullptr)
        return;
      if (strcmp(sub.type->name, "std_msgs/Float32") != 0)
        return;
      float rad = static_cast<const Float32Msg *>(sample)->data;
      float level = rad_to_level_(rad, sub.min_rad, sub.max_rad);
      sub.servo->write(level);
      this->remember_level_(sub.servo, level);
#else
      (void)sub;
      (void)sample;
#endif
    }

    void Ros2Component::dispatch_light_(const Subscription &sub, const void *sample)
    {
#ifdef USE_LIGHT
      if (sub.light == nullptr || sub.type == nullptr)
        return;
      float r = 0.0f, g = 0.0f, b = 0.0f;
      bool have_color = false;
      bool off = false;
      if (strcmp(sub.type->name, "std_msgs/ColorRGBA") == 0)
      {
        auto *msg = static_cast<const ColorRGBAMsg *>(sample);
        if (sub.light_field == LightField::BRIGHTNESS)
        {
          auto call = sub.light->turn_on();
          call.set_brightness(clamp01_(msg->a));
          call.perform();
          return;
        }
        r = clamp01_(msg->r);
        g = clamp01_(msg->g);
        b = clamp01_(msg->b);
        have_color = true;
      }
      else if (strcmp(sub.type->name, "sensor_msgs/Joy") == 0)
      {
        auto *msg = static_cast<const JoyMsg *>(sample);
        if (msg->num_buttons > 0 && msg->buttons[0] == 0)
          off = true;
        if (sub.light_field == LightField::BRIGHTNESS)
        {
          float v = msg->num_axes > 0 ? (msg->axes[0] + 1.0f) * 0.5f : 0.0f;
          if (off)
          {
            sub.light->make_call().set_state(false).perform();
            return;
          }
          auto call = sub.light->turn_on();
          call.set_brightness(clamp01_(v));
          call.perform();
          return;
        }
        r = msg->num_axes > 0 ? clamp01_((msg->axes[0] + 1.0f) * 0.5f) : 0.0f;
        g = msg->num_axes > 1 ? clamp01_((msg->axes[1] + 1.0f) * 0.5f) : 0.0f;
        b = msg->num_axes > 2 ? clamp01_((msg->axes[2] + 1.0f) * 0.5f) : 0.0f;
        have_color = true;
      }
      else
      {
        return;
      }
      if (!have_color)
        return;
      if (off)
      {
        sub.light->make_call().set_state(false).perform();
        return;
      }
      auto call = sub.light->turn_on();
      call.set_rgb(r, g, b);
      call.perform();
#else
      (void)sub;
      (void)sample;
#endif
    }

    float Ros2Component::clamp01_(float v)
    {
      if (v > 1.0f)
        return 1.0f;
      if (v < 0.0f)
        return 0.0f;
      return v;
    }

    LightField Ros2Component::parse_light_field_(const char *field)
    {
      if (field != nullptr && strcmp(field, "brightness") == 0)
        return LightField::BRIGHTNESS;
      return LightField::RGB;
    }

    void Ros2Component::dispatch_joints_(const Subscription &sub, const void *sample)    {
#ifdef USE_SERVO
      if (sub.type == nullptr)
        return;
      const char *names[ROS2_MAX_JOINTS]{nullptr};
      const float *positions = nullptr;
      uint8_t n = 0;
      float traj_storage[ROS2_MAX_JOINTS]{0.0f};
      if (strcmp(sub.type->name, "sensor_msgs/JointState") == 0)
      {
        auto *msg = static_cast<const JointStateMsg *>(sample);
        n = msg->num_joints;
        positions = msg->position;
        for (uint8_t i = 0; i < n && i < ROS2_MAX_JOINTS; i++)
          names[i] = msg->name[i];
      }
      else if (strcmp(sub.type->name, "trajectory_msgs/JointTrajectory") == 0)
      {
        auto *msg = static_cast<const JointTrajectoryMsg *>(sample);
        if (msg->num_points == 0)
          return;
        n = msg->num_joints;
        for (uint8_t i = 0; i < n && i < ROS2_MAX_JOINTS; i++)
        {
          names[i] = msg->joint_names[i];
          traj_storage[i] = msg->points[0].positions[i];
        }
        positions = traj_storage;
      }
      else
      {
        return;
      }
      for (uint8_t i = 0; i < n && i < ROS2_MAX_JOINTS; i++)
      {
        for (size_t t = 0; t < sub.num_joints; t++)
        {
          if (strcmp(sub.joints[t].joint_name, names[i]) == 0)
          {
            float level = rad_to_level_(positions[i], sub.joints[t].min_rad, sub.joints[t].max_rad);
            if (sub.joints[t].servo != nullptr)
            {
              sub.joints[t].servo->write(level);
              this->remember_level_(sub.joints[t].servo, level);
            }
            break;
          }
        }
      }
#else
      (void)sub;
      (void)sample;
#endif
    }

    void Ros2Component::dispatch_diff_drive_(const Subscription &sub, const void *sample)
    {
#ifdef USE_SERVO
      if (sub.type == nullptr || strcmp(sub.type->name, "geometry_msgs/Twist") != 0)
        return;
      if (sub.left_wheel == nullptr || sub.right_wheel == nullptr)
        return;
      auto *msg = static_cast<const TwistMsg *>(sample);
      // Planar only: linear.y/z and angular.x/y are ignored by design.
      float v = msg->linear_x;
      if (v > sub.max_linear_speed)
        v = sub.max_linear_speed;
      if (v < -sub.max_linear_speed)
        v = -sub.max_linear_speed;
      float w = msg->angular_z;
      if (w > sub.max_angular_speed)
        w = sub.max_angular_speed;
      if (w < -sub.max_angular_speed)
        w = -sub.max_angular_speed;
      const float vl = v - w * sub.wheel_separation * 0.5f;
      const float vr = v + w * sub.wheel_separation * 0.5f;
      float scale = sub.max_linear_speed + sub.max_angular_speed * sub.wheel_separation * 0.5f;
      if (scale <= 0.0f)
        scale = 1.0f;
      float ll = vl / scale;
      float rl = vr / scale;
      if (ll > 1.0f)
        ll = 1.0f;
      if (ll < -1.0f)
        ll = -1.0f;
      if (rl > 1.0f)
        rl = 1.0f;
      if (rl < -1.0f)
        rl = -1.0f;
      sub.left_wheel->write(ll);
      sub.right_wheel->write(rl);
      this->cmd_vl_ = vl;
      this->cmd_vr_ = vr;
      this->cmd_time_ = App.get_loop_component_start_time();
      this->cmd_timeout_ms_ = sub.cmd_timeout_ms;
      this->cmd_active_ = true;
#else
      (void) sub;
      (void) sample;
#endif
    }

    void Ros2Component::stop_stale_diff_drive_(uint32_t now)
    {
#ifdef USE_SERVO
      if (!this->cmd_active_ || now - this->cmd_time_ < this->cmd_timeout_ms_)
        return;
      this->cmd_active_ = false;
      this->cmd_vl_ = 0.0f;
      this->cmd_vr_ = 0.0f;
      for (size_t i = 0; i < this->num_subs_; i++)
      {
        if (this->subs_[i].kind != SubKind::DIFF_DRIVE)
          continue;
        if (this->subs_[i].left_wheel != nullptr)
          this->subs_[i].left_wheel->write(0.0f);
        if (this->subs_[i].right_wheel != nullptr)
          this->subs_[i].right_wheel->write(0.0f);
      }
#else
      (void) now;
#endif
    }

    float Ros2Component::rad_to_level_(float rad, float min_rad, float max_rad)
    {
      if (max_rad <= min_rad)
        return 0.0f;
      float level = (rad - min_rad) / (max_rad - min_rad) * 2.0f - 1.0f;
      if (level > 1.0f)
        level = 1.0f;
      if (level < -1.0f)
        level = -1.0f;
      return level;
    }

    float Ros2Component::level_to_rad_(float level, float min_rad, float max_rad)
    {
      return min_rad + (level + 1.0f) * 0.5f * (max_rad - min_rad);
    }

    void Ros2Component::remember_level_(servo::Servo *servo, float level)
    {
      for (size_t i = 0; i < this->num_levels_; i++)
      {
        if (this->levels_[i].servo == servo)
        {
          this->levels_[i].level = level;
          return;
        }
      }
      if (this->num_levels_ < this->levels_.size())
        this->levels_[this->num_levels_++] = ServoLevel{servo, level};
    }

    float Ros2Component::recalled_level_(servo::Servo *servo)
    {
      for (size_t i = 0; i < this->num_levels_; i++)
      {
        if (this->levels_[i].servo == servo)
          return this->levels_[i].level;
      }
      return 0.0f;
    }

    void Ros2Component::add_servo_subscription(const char *topic, const char *type, servo::Servo *servo, float min_rad,
                                               float max_rad)
    {
#ifdef USE_SERVO
      if (this->num_subs_ >= ROS2_MAX_SUBSCRIPTIONS)
      {
        ESP_LOGE(TAG, "Too many subscriptions (max %u)", (unsigned)ROS2_MAX_SUBSCRIPTIONS);
        return;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return;
      }
      Subscription sub;
      sub.topic = topic;
      sub.type = def;
      sub.kind = SubKind::SERVO_SINGLE;
      sub.servo = servo;
      sub.min_rad = min_rad;
      sub.max_rad = max_rad;
      this->subs_[this->num_subs_++] = sub;
#else
      (void)topic;
      (void)type;
      (void)servo;
      (void)min_rad;
      (void)max_rad;
      ESP_LOGE(TAG, "servo support not compiled in");
#endif
    }

    void Ros2Component::add_joint_subscription(const char *topic, const char *type, servo::Servo *servo,
                                               const char *joint_name, float min_rad, float max_rad)
    {
#ifdef USE_SERVO
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return;
      }
      if (!is_multi_joint_type(def))
      {
        ESP_LOGE(TAG, "Type %s is not multi-joint; use target: (singular)", type);
        return;
      }
      for (size_t i = 0; i < this->num_subs_; i++)
      {
        if (this->subs_[i].topic == topic)
        {
          if (this->subs_[i].num_joints >= ROS2_MAX_TARGETS)
          {
            ESP_LOGE(TAG, "Too many targets for %s", topic);
            return;
          }
          JointTarget t;
          strncpy(t.joint_name, joint_name, ROS2_NAME_LEN - 1);
          t.servo = servo;
          t.min_rad = min_rad;
          t.max_rad = max_rad;
          this->subs_[i].joints[this->subs_[i].num_joints++] = t;
          return;
        }
      }
      if (this->num_subs_ >= ROS2_MAX_SUBSCRIPTIONS)
      {
        ESP_LOGE(TAG, "Too many subscriptions (max %u)", (unsigned)ROS2_MAX_SUBSCRIPTIONS);
        return;
      }
      Subscription sub;
      sub.topic = topic;
      sub.type = def;
      sub.kind = SubKind::JOINT_MULTI;
      JointTarget t;
      strncpy(t.joint_name, joint_name, ROS2_NAME_LEN - 1);
      t.servo = servo;
      t.min_rad = min_rad;
      t.max_rad = max_rad;
      sub.joints[0] = t;
      sub.num_joints = 1;
      this->subs_[this->num_subs_++] = sub;
#else
      (void)topic;
      (void)type;
      (void)servo;
      (void)joint_name;
      (void)min_rad;
      (void)max_rad;
      ESP_LOGE(TAG, "servo support not compiled in");
#endif
    }

    void Ros2Component::add_joint_state_source(servo::Servo *servo, const char *joint_name,
                                               float min_rad, float max_rad)
    {
#ifdef USE_SERVO
      if (this->num_pubs_ == 0)
        return;
      Publication &pub = this->pubs_[this->num_pubs_ - 1];
      if (pub.kind != PubKind::JOINT_MULTI)
        return;
      if (pub.num_joints >= ROS2_MAX_TARGETS)
        return;
      JointSource s;
      strncpy(s.joint_name, joint_name, ROS2_NAME_LEN - 1);
      s.servo = servo;
      s.min_rad = min_rad;
      s.max_rad = max_rad;
      pub.joints[pub.num_joints++] = s;
#else
      (void)servo;
      (void)joint_name;
      (void)min_rad;
      (void)max_rad;
      ESP_LOGE(TAG, "servo support not compiled in");
#endif
    }

    void Ros2Component::add_switch_subscription(const char *topic, const char *type, switch_::Switch *sw)
    {
#ifndef USE_SWITCH
      (void)topic;
      (void)type;
      (void)sw;
      ESP_LOGE(TAG, "switch support not compiled in");
      return;
#else
      if (this->num_subs_ >= ROS2_MAX_SUBSCRIPTIONS)
      {
        ESP_LOGE(TAG, "Too many subscriptions (max %u)", (unsigned)ROS2_MAX_SUBSCRIPTIONS);
        return;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return;
      }
      if (strcmp(def->name, "std_msgs/Bool") != 0)
      {
        ESP_LOGE(TAG, "Switch targets require std_msgs/Bool (got %s)", type);
        return;
      }
      Subscription sub;
      sub.topic = topic;
      sub.type = def;
      sub.kind = SubKind::SWITCH_SINGLE;
      sub.sw = sw;
      this->subs_[this->num_subs_++] = sub;
#endif
    }

    uint8_t Ros2Component::add_switch_publication(const char *topic, const char *type, switch_::Switch *sw,
                                                  uint32_t interval_ms)
    {
#ifndef USE_SWITCH
      (void)topic;
      (void)type;
      (void)sw;
      (void)interval_ms;
      ESP_LOGE(TAG, "switch support not compiled in");
      return 255;
#else
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return 255;
      }
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::SWITCH_SINGLE;
      pub.sw = sw;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#endif
    }

    uint8_t Ros2Component::add_sensor_publication(const char *topic, const char *type, sensor::Sensor *sensor,
                                                  uint32_t interval_ms)
    {
#ifndef USE_SENSOR
      (void)topic;
      (void)type;
      (void)sensor;
      (void)interval_ms;
      ESP_LOGE(TAG, "sensor support not compiled in");
      return 255;
#else
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return 255;
      }
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::SENSOR_SINGLE;
      pub.sensor = sensor;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#endif
    }

    uint8_t Ros2Component::add_binary_sensor_publication(const char *topic, const char *type,
                                                         binary_sensor::BinarySensor *bs, uint32_t interval_ms)
    {
#ifndef USE_BINARY_SENSOR
      (void)topic;
      (void)type;
      (void)bs;
      (void)interval_ms;
      ESP_LOGE(TAG, "binary_sensor support not compiled in");
      return 255;
#else
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return 255;
      }
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::BINARY_SENSOR_SINGLE;
      pub.bsensor = bs;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#endif
    }

    uint8_t Ros2Component::add_joint_state_publication(const char *topic, const char *type, uint32_t interval_ms)
    {
#ifdef USE_SERVO
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr || !is_multi_joint_type(def))
      {
        ESP_LOGE(TAG, "Joint publication needs a multi-joint type (got '%s')", type);
        return 255;
      }
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::JOINT_MULTI;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#else
      (void)topic;
      (void)type;
      (void)interval_ms;
      ESP_LOGE(TAG, "servo support not compiled in");
      return 255;
#endif
    }

    void Ros2Component::add_light_subscription(const char *topic, const char *type, light::LightState *light,
                                               const char *field)
    {
#ifdef USE_LIGHT
      if (this->num_subs_ >= ROS2_MAX_SUBSCRIPTIONS)
      {
        ESP_LOGE(TAG, "Too many subscriptions (max %u)", (unsigned)ROS2_MAX_SUBSCRIPTIONS);
        return;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr)
      {
        ESP_LOGE(TAG, "Unknown type '%s' for topic %s", type, topic);
        return;
      }
      if (strcmp(def->name, "std_msgs/ColorRGBA") != 0 && strcmp(def->name, "sensor_msgs/Joy") != 0)
      {
        ESP_LOGE(TAG, "Light targets need std_msgs/ColorRGBA or sensor_msgs/Joy (got %s)", type);
        return;
      }
      Subscription sub;
      sub.topic = topic;
      sub.type = def;
      sub.kind = SubKind::LIGHT_SINGLE;
      sub.light = light;
      sub.light_field = parse_light_field_(field);
      this->subs_[this->num_subs_++] = sub;
#else
      (void)topic;
      (void)type;
      (void)light;
      (void)field;
      ESP_LOGE(TAG, "light support not compiled in");
#endif
    }

    void Ros2Component::add_diff_drive_subscription(const char *topic, servo::Servo *left, servo::Servo *right,
                                                     float wheel_separation, float max_linear_speed,
                                                     float max_angular_speed, uint32_t cmd_timeout_ms)
    {
#ifdef USE_SERVO
      if (this->num_subs_ >= ROS2_MAX_SUBSCRIPTIONS)
      {
        ESP_LOGE(TAG, "Too many subscriptions (max %u)", (unsigned) ROS2_MAX_SUBSCRIPTIONS);
        return;
      }
      const TypeDef *def = find_type("geometry_msgs/Twist");
      if (def == nullptr)
        return;
      if (left == nullptr || right == nullptr)
      {
        ESP_LOGE(TAG, "Diff-drive needs left: and right: servos for %s", topic);
        return;
      }
      Subscription sub;
      sub.topic = topic;
      sub.type = def;
      sub.kind = SubKind::DIFF_DRIVE;
      sub.left_wheel = left;
      sub.right_wheel = right;
      sub.wheel_separation = wheel_separation;
      sub.max_linear_speed = max_linear_speed;
      sub.max_angular_speed = max_angular_speed;
      sub.cmd_timeout_ms = cmd_timeout_ms != 0 ? cmd_timeout_ms : 500;
      this->subs_[this->num_subs_++] = sub;
#else
      (void) topic;
      (void) left;
      (void) right;
      (void) wheel_separation;
      (void) max_linear_speed;
      (void) max_angular_speed;
      (void) cmd_timeout_ms;
      ESP_LOGE(TAG, "servo support not compiled in");
#endif
    }

    uint8_t Ros2Component::add_light_publication(const char *topic, const char *type, light::LightState *light,
                                                  uint32_t interval_ms)
    {
#ifdef USE_LIGHT
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr || strcmp(def->name, "std_msgs/ColorRGBA") != 0)
      {
        ESP_LOGE(TAG, "Light publication needs std_msgs/ColorRGBA (got '%s')", type);
        return 255;
      }
      if (light == nullptr)
      {
        ESP_LOGE(TAG, "Light publication needs a light (got null)");
        return 255;
      }
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::LIGHT_SINGLE;
      pub.light = light;
      pub.light_field = LightField::RGB;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#else
      (void)topic;
      (void)type;
      (void)light;
      (void)interval_ms;
      ESP_LOGE(TAG, "light support not compiled in");
      return 255;
#endif
    }

    uint8_t Ros2Component::add_image_publication(const char *topic, const char *type, camera::Camera *camera,
                                                 uint32_t interval_ms)
    {
#ifdef USE_CAMERA
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned) ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type(type);
      if (def == nullptr || strcmp(def->name, "sensor_msgs/CompressedImage") != 0)
      {
        ESP_LOGE(TAG, "Image publication needs sensor_msgs/CompressedImage (got '%s')", type);
        return 255;
      }
      if (camera == nullptr)
      {
        ESP_LOGE(TAG, "Image publication needs a camera (got null)");
        return 255;
      }
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::IMAGE_SINGLE;
      pub.camera = camera;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#else
      (void)topic;
      (void)type;
      (void)camera;
      (void)interval_ms;
      ESP_LOGE(TAG, "camera support not compiled in");
      return 255;
#endif
    }

    void Ros2Component::set_subscription_qos(const char *topic, const char *qos)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_subs_; i++)
      {
        if (this->subs_[i].topic == topic)
          apply_qos_(this->subs_[i].reliable, this->subs_[i].qos_explicit, qos);
      }
    }

    void Ros2Component::set_publication_qos(const char *topic, const char *qos)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic)
          apply_qos_(this->pubs_[i].reliable, this->pubs_[i].qos_explicit, qos);
      }
    }

    void Ros2Component::set_publication_frame_id(const char *topic, const char *frame_id)
    {
      if (topic == nullptr || frame_id == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic)
        {
          strncpy(this->pubs_[i].frame_id, frame_id, ROS2_FRAME_ID_LEN - 1);
          this->pubs_[i].frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
        }
      }
    }

    void Ros2Component::set_range_params(const char *topic, uint8_t radiation_type, float field_of_view,
                                         float min_range, float max_range, float variance)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic)
        {
          this->pubs_[i].radiation_type = radiation_type;
          this->pubs_[i].field_of_view = field_of_view;
          this->pubs_[i].min_range = min_range;
          this->pubs_[i].max_range = max_range;
          this->pubs_[i].range_variance = variance;
        }
      }
    }

    void Ros2Component::set_battery_params(const char *topic, float min_voltage, float max_voltage,
                                           float design_capacity, uint8_t technology, const char *location)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic)
        {
          this->pubs_[i].min_voltage = min_voltage;
          this->pubs_[i].max_voltage = max_voltage;
          this->pubs_[i].design_capacity = design_capacity;
          this->pubs_[i].battery_technology = technology;
          if (location != nullptr)
          {
            strncpy(this->pubs_[i].battery_location, location, ROS2_NAME_LEN - 1);
            this->pubs_[i].battery_location[ROS2_NAME_LEN - 1] = '\0';
          }
        }
      }
    }

    uint8_t Ros2Component::add_range_publication(const char *topic, sensor::Sensor *sensor, uint32_t interval_ms)
    {
#ifndef USE_SENSOR
      (void) topic;
      (void) sensor;
      (void) interval_ms;
      ESP_LOGE(TAG, "sensor support not compiled in");
      return 255;
#else
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned) ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type("sensor_msgs/Range");
      if (def == nullptr)
        return 255;
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::SENSOR_SINGLE;
      pub.sensor = sensor;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#endif
    }

    uint8_t Ros2Component::add_battery_publication(const char *topic, sensor::Sensor *sensor, uint32_t interval_ms)
    {
#ifndef USE_SENSOR
      (void) topic;
      (void) sensor;
      (void) interval_ms;
      ESP_LOGE(TAG, "sensor support not compiled in");
      return 255;
#else
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned) ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type("sensor_msgs/BatteryState");
      if (def == nullptr)
        return 255;
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::SENSOR_SINGLE;
      pub.sensor = sensor;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#endif
    }

    void Ros2Component::set_odom_params(const char *topic, float wheel_separation,
                                        const char *child_frame_id, const char *tf_topic)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic)
        {
          this->pubs_[i].odom_wheel_separation = wheel_separation;
          if (child_frame_id != nullptr)
          {
            strncpy(this->pubs_[i].child_frame_id, child_frame_id, ROS2_FRAME_ID_LEN - 1);
            this->pubs_[i].child_frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
          }
          if (tf_topic != nullptr)
            this->pubs_[i].tf_topic = tf_topic;
        }
      }
    }

    uint8_t Ros2Component::add_odom_publication(const char *topic, uint32_t interval_ms)
    {
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned) ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type("nav_msgs/Odometry");
      if (def == nullptr)
        return 255;
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::ODOM;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      strncpy(pub.child_frame_id, "base_link", ROS2_FRAME_ID_LEN - 1);
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
    }

    uint8_t Ros2Component::add_tf_publication(const char *topic, uint32_t interval_ms)
    {
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type("tf2_msgs/TFMessage");
      if (def == nullptr)
        return 255;
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::TF;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
    }

    uint8_t Ros2Component::add_imu_publication(const char *topic, uint32_t interval_ms)
    {
#ifndef USE_SENSOR
      (void) topic;
      (void) interval_ms;
      ESP_LOGE(TAG, "sensor support not compiled in");
      return 255;
#else
      if (this->num_pubs_ >= ROS2_MAX_PUBLICATIONS)
      {
        ESP_LOGE(TAG, "Too many publications (max %u)", (unsigned)ROS2_MAX_PUBLICATIONS);
        return 255;
      }
      const TypeDef *def = find_type("sensor_msgs/Imu");
      if (def == nullptr)
        return 255;
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::IMU;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
#endif
    }

    void Ros2Component::set_imu_sources(const char *topic, sensor::Sensor *ax, sensor::Sensor *ay,
                                        sensor::Sensor *az, sensor::Sensor *gx, sensor::Sensor *gy,
                                        sensor::Sensor *gz)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic && this->pubs_[i].kind == PubKind::IMU)
        {
          this->pubs_[i].imu_accel[0] = ax;
          this->pubs_[i].imu_accel[1] = ay;
          this->pubs_[i].imu_accel[2] = az;
          this->pubs_[i].imu_gyro[0] = gx;
          this->pubs_[i].imu_gyro[1] = gy;
          this->pubs_[i].imu_gyro[2] = gz;
        }
      }
    }

    void Ros2Component::set_imu_orientation(const char *topic, sensor::Sensor *ox, sensor::Sensor *oy,
                                            sensor::Sensor *oz, sensor::Sensor *ow)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        if (this->pubs_[i].topic == topic && this->pubs_[i].kind == PubKind::IMU)
        {
          this->pubs_[i].imu_orientation[0] = ox;
          this->pubs_[i].imu_orientation[1] = oy;
          this->pubs_[i].imu_orientation[2] = oz;
          this->pubs_[i].imu_orientation[3] = ow;
          this->pubs_[i].imu_has_orientation = true;
        }
      }
    }
      const TypeDef *def = find_type("tf2_msgs/TFMessage");
      if (def == nullptr)
        return 255;
      Publication pub;
      pub.topic = topic;
      pub.type = def;
      pub.kind = PubKind::TF;
      pub.interval_ms = interval_ms != 0 ? interval_ms : this->default_interval_ms_;
      this->pubs_[this->num_pubs_] = pub;
      return this->num_pubs_++;
    }

    void Ros2Component::add_tf_transform(const char *topic, const char *frame_id, const char *child_frame_id,
                                         float tx, float ty, float tz, float qx, float qy, float qz,
                                         float qw)
    {
      if (topic == nullptr)
        return;
      for (size_t i = 0; i < this->num_pubs_; i++)
      {
        Publication &pub = this->pubs_[i];
        if (pub.topic != topic || pub.kind != PubKind::TF)
          continue;
        if (pub.num_tf_transforms >= ROS2_MAX_TF_TRANSFORMS)
        {
          ESP_LOGE(TAG, "Too many transforms for %s (max %u)", topic,
                   (unsigned) ROS2_MAX_TF_TRANSFORMS);
          return;
        }
        TFTransformMsg &t = pub.tf_transforms[pub.num_tf_transforms++];
        memset(&t, 0, sizeof(t));
        if (frame_id != nullptr)
        {
          strncpy(t.header.frame_id, frame_id, ROS2_FRAME_ID_LEN - 1);
          t.header.frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
        }
        if (child_frame_id != nullptr)
        {
          strncpy(t.child_frame_id, child_frame_id, ROS2_FRAME_ID_LEN - 1);
          t.child_frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
        }
        t.translation[0] = tx;
        t.translation[1] = ty;
        t.translation[2] = tz;
        // Normalize defensively: a non-unit user quaternion would silently
        // corrupt every downstream TF lookup.
        float n = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
        if (n > 0.0f)
        {
          t.rotation[0] = qx / n;
          t.rotation[1] = qy / n;
          t.rotation[2] = qz / n;
          t.rotation[3] = qw / n;
        }
        else
        {
          t.rotation[3] = 1.0f;
        }
      }
    }

    void Ros2Component::poll_publication_(Publication &pub)
    {
      if (this->mw_ == nullptr || !this->mw_->connected())
        return;
      if (pub.kind == PubKind::IMAGE_SINGLE)
        return;  // push path: on_camera_image publishes, throttled by interval_ms
      const uint32_t now = App.get_loop_component_start_time();
      if (now - pub.last_pub < pub.interval_ms)
        return;
      pub.last_pub = now;
      if (pub.type == nullptr)
        return;
      MiddlewareOptions opts;
      opts.reliable = pub.reliable;
      opts.qos_explicit = pub.qos_explicit;
      if (strcmp(pub.type->name, "std_msgs/Float32") == 0)
      {
#ifdef USE_SENSOR
        if (pub.sensor == nullptr || !pub.sensor->has_state())
          return;
        Float32Msg msg;
        msg.data = pub.sensor->state;
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
#endif
      }
      else if (strcmp(pub.type->name, "std_msgs/Bool") == 0)
      {
        BoolMsg msg;
        msg.data = false;
        bool have = false;
#ifdef USE_SWITCH
        if (pub.kind == PubKind::SWITCH_SINGLE && pub.sw != nullptr)
        {
          msg.data = pub.sw->state;
          have = true;
        }
#endif
#ifdef USE_BINARY_SENSOR
        if (pub.kind == PubKind::BINARY_SENSOR_SINGLE && pub.bsensor != nullptr)
        {
          msg.data = pub.bsensor->state;
          have = true;
        }
#endif
        if (have)
          this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
      }
      else if (strcmp(pub.type->name, "sensor_msgs/JointState") == 0)
      {
        JointStateMsg msg;
        memset(&msg, 0, sizeof(msg));
        this->fill_header_(msg.header, pub.frame_id);
        uint8_t n = pub.num_joints > ROS2_MAX_JOINTS ? ROS2_MAX_JOINTS : pub.num_joints;
        msg.num_joints = n;
        for (uint8_t i = 0; i < n; i++)
        {
          strncpy(msg.name[i], pub.joints[i].joint_name, ROS2_NAME_LEN - 1);
          float level = pub.joints[i].servo != nullptr ? this->recalled_level_(pub.joints[i].servo) : 0.0f;
          msg.position[i] = level_to_rad_(level, pub.joints[i].min_rad, pub.joints[i].max_rad);
        }
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
      }
      else if (strcmp(pub.type->name, "sensor_msgs/Range") == 0)
      {
#ifdef USE_SENSOR
        if (pub.sensor == nullptr || !pub.sensor->has_state())
          return;
        RangeMsg msg;
        memset(&msg, 0, sizeof(msg));
        this->fill_header_(msg.header, pub.frame_id);
        msg.radiation_type = pub.radiation_type;
        msg.field_of_view = pub.field_of_view;
        msg.min_range = pub.min_range;
        msg.max_range = pub.max_range;
        msg.range = pub.sensor->state;
        msg.variance = pub.range_variance;
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
#endif
      }
      else if (strcmp(pub.type->name, "sensor_msgs/BatteryState") == 0)
      {
#ifdef USE_SENSOR
        if (pub.sensor == nullptr || !pub.sensor->has_state())
          return;
        BatteryStateMsg msg;
        memset(&msg, 0, sizeof(msg));
        this->fill_header_(msg.header, pub.frame_id);
        const float volts = pub.sensor->state;
        msg.voltage = volts;
        msg.temperature = NAN;
        msg.current = NAN;
        msg.charge = NAN;
        msg.capacity = NAN;
        msg.design_capacity = pub.design_capacity > 0.0f ? pub.design_capacity : NAN;
        if (pub.max_voltage > pub.min_voltage)
        {
          float pct = (volts - pub.min_voltage) / (pub.max_voltage - pub.min_voltage);
          if (pct < 0.0f)
            pct = 0.0f;
          if (pct > 1.0f)
            pct = 1.0f;
          msg.percentage = pct;
        }
        else
        {
          msg.percentage = NAN;
        }
        msg.power_supply_status = 0;  // UNKNOWN: the bridge cannot see the charger
        msg.power_supply_health = 0;  // UNKNOWN
        msg.power_supply_technology = pub.battery_technology;
        msg.present = true;
        strncpy(msg.location, pub.battery_location, ROS2_NAME_LEN - 1);
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
#endif
      }
      else if (strcmp(pub.type->name, "nav_msgs/Odometry") == 0)
      {
        this->poll_odom_(pub, opts, now);
      }
      else if (strcmp(pub.type->name, "tf2_msgs/TFMessage") == 0)
      {
        this->poll_tf_(pub, opts);
      }
      else if (strcmp(pub.type->name, "sensor_msgs/Imu") == 0)
      {
        this->poll_imu_(pub, opts);
      }
      else if (strcmp(pub.type->name, "std_msgs/ColorRGBA") == 0)
      {
#ifdef USE_LIGHT
        if (pub.kind != PubKind::LIGHT_SINGLE || pub.light == nullptr)
          return;
        ColorRGBAMsg msg;
        if (pub.light_field == LightField::BRIGHTNESS)
        {
          msg.a = pub.light->remote_values.get_brightness();
        }
        else
        {
          msg.r = pub.light->remote_values.get_red();
          msg.g = pub.light->remote_values.get_green();
          msg.b = pub.light->remote_values.get_blue();
          msg.a = pub.light->remote_values.get_brightness();
        }
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
#endif
      }
    }

    void Ros2Component::poll_odom_(Publication &pub, const MiddlewareOptions &opts, uint32_t now)
    {
      if (pub.kind != PubKind::ODOM)
        return;
      // Open-loop dead reckoning from the last commanded wheel velocities.
      // Stale commands (cmd_vel timeout) integrate as zero, never as the
      // last value: a silent base must read stopped, not drifting.
      float vl = 0.0f;
      float vr = 0.0f;
      if (this->cmd_active_)
      {
        vl = this->cmd_vl_;
        vr = this->cmd_vr_;
      }
      float dt = 0.0f;
      if (pub.odom_last_ms != 0)
        dt = (now - pub.odom_last_ms) / 1000.0f;
      pub.odom_last_ms = now;
      if (dt < 0.0f)
        dt = 0.0f;
      if (dt > 1.0f)
        dt = 1.0f;
      float sep = pub.odom_wheel_separation;
      if (sep <= 0.0f)
        sep = 0.2f;
      const float v = (vl + vr) * 0.5f;
      const float w = (vr - vl) / sep;
      pub.odom_x += v * cosf(pub.odom_theta) * dt;
      pub.odom_y += v * sinf(pub.odom_theta) * dt;
      pub.odom_theta += w * dt;
      while (pub.odom_theta > 3.14159265f)
        pub.odom_theta -= 6.2831853f;
      while (pub.odom_theta < -3.14159265f)
        pub.odom_theta += 6.2831853f;

      OdometryMsg msg;
      memset(&msg, 0, sizeof(msg));
      this->fill_header_(msg.header, pub.frame_id);
      strncpy(msg.child_frame_id, pub.child_frame_id, ROS2_FRAME_ID_LEN - 1);
      msg.pose_position[0] = pub.odom_x;
      msg.pose_position[1] = pub.odom_y;
      const float half = pub.odom_theta * 0.5f;
      msg.pose_orientation[2] = sinf(half);
      msg.pose_orientation[3] = cosf(half);
      msg.twist_linear[0] = v;
      msg.twist_angular[2] = w;
      this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
      if (!pub.tf_topic.empty())
      {
        this->publish_tf_transform_(pub.tf_topic, opts, msg.header.stamp_sec, pub.frame_id,
                                    pub.child_frame_id, pub.odom_x, pub.odom_y,
                                    msg.pose_orientation[2], msg.pose_orientation[3]);
      }
    }

    void Ros2Component::publish_tf_transform_(const std::string &topic, const MiddlewareOptions &opts,
                                              int32_t sec, const char *frame_id, const char *child_frame_id,
                                              float x, float y, float qz, float qw)
    {
      const TypeDef *def = find_type("tf2_msgs/TFMessage");
      if (def == nullptr)
        return;
      TFMessageMsg msg;
      memset(&msg, 0, sizeof(msg));
      msg.num_transforms = 1;
      TFTransformMsg &t = msg.transforms[0];
      t.header.stamp_sec = sec;
      if (frame_id != nullptr)
      {
        strncpy(t.header.frame_id, frame_id, ROS2_FRAME_ID_LEN - 1);
        t.header.frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
      }
      if (child_frame_id != nullptr)
      {
        strncpy(t.child_frame_id, child_frame_id, ROS2_FRAME_ID_LEN - 1);
        t.child_frame_id[ROS2_FRAME_ID_LEN - 1] = '\0';
      }
      t.translation[0] = x;
      t.translation[1] = y;
      t.rotation[2] = qz;
      t.rotation[3] = qw;
      this->mw_->publish(topic, def, &msg, sizeof(msg), &opts);
    }

    void Ros2Component::poll_tf_(Publication &pub, const MiddlewareOptions &opts)
    {
      if (pub.kind != PubKind::TF || pub.num_tf_transforms == 0)
        return;
      TFMessageMsg msg;
      memset(&msg, 0, sizeof(msg));
      msg.num_transforms = pub.num_tf_transforms;
      for (uint8_t i = 0; i < pub.num_tf_transforms; i++)
      {
        msg.transforms[i] = pub.tf_transforms[i];
        this->fill_header_(msg.transforms[i].header, pub.tf_transforms[i].header.frame_id);
      }
      this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
    }

    void Ros2Component::poll_imu_(Publication &pub, const MiddlewareOptions &opts)
    {
      if (pub.kind != PubKind::IMU)
        return;
#ifdef USE_SENSOR
      // All six accel/gyro axes are required; skip until every axis has
      // state (mirrors Range/BatteryState). Orientation is optional.
      for (uint8_t i = 0; i < 3; i++)
      {
        if (pub.imu_accel[i] == nullptr || !pub.imu_accel[i]->has_state())
          return;
        if (pub.imu_gyro[i] == nullptr || !pub.imu_gyro[i]->has_state())
          return;
      }
      ImuMsg msg;
      memset(&msg, 0, sizeof(msg));
      this->fill_header_(msg.header, pub.frame_id);
      msg.linear_acceleration[0] = pub.imu_accel[0]->state;
      msg.linear_acceleration[1] = pub.imu_accel[1]->state;
      msg.linear_acceleration[2] = pub.imu_accel[2]->state;
      msg.angular_velocity[0] = pub.imu_gyro[0]->state;
      msg.angular_velocity[1] = pub.imu_gyro[1]->state;
      msg.angular_velocity[2] = pub.imu_gyro[2]->state;
      // Covariances unknown: zeros. Orientation without a source publishes
      // 0,0,0,0 with covariance[0] = -1 ("no estimate") per the IDL.
      msg.orientation_covariance[0] = -1.0f;
      bool have_orientation = pub.imu_has_orientation;
      if (have_orientation)
      {
        for (uint8_t i = 0; i < 4; i++)
        {
          if (pub.imu_orientation[i] == nullptr || !pub.imu_orientation[i]->has_state())
          {
            have_orientation = false;
            break;
          }
        }
      }
      if (have_orientation)
      {
        float qx = pub.imu_orientation[0]->state;
        float qy = pub.imu_orientation[1]->state;
        float qz = pub.imu_orientation[2]->state;
        float qw = pub.imu_orientation[3]->state;
        float n = sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
        if (n > 0.0f)
        {
          msg.orientation[0] = qx / n;
          msg.orientation[1] = qy / n;
          msg.orientation[2] = qz / n;
          msg.orientation[3] = qw / n;
        }
        else
        {
          msg.orientation[3] = 1.0f;
        }
        msg.orientation_covariance[0] = 0.0f;
      }
      this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg), &opts);
#else
      (void) pub;
      (void) opts;
#endif
    }

  } // namespace ros2
} // namespace esphome

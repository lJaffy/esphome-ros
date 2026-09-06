#include "ros2_component.h"

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
      if (!this->mw_->publish_image(pub.topic, jpeg, len))
        ESP_LOGW(TAG, "Image publish failed for %s (%u bytes)", pub.topic.c_str(), (unsigned) len);
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
            });
        ESP_LOGI(TAG, "Subscribed: %s (%s)", sub.topic.c_str(), sub.type->name);
      }
      this->subscribed_ = true;
    }

    void Ros2Component::loop()
    {
      this->try_subscribe_();
      const uint32_t now = App.get_loop_component_start_time();
      for (size_t i = 0; i < this->num_pubs_; i++)
        this->poll_publication_(this->pubs_[i]);
      (void)now;
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
      if (sub.servo == nullptr || sub.type == nullptr)
        return;
      if (strcmp(sub.type->name, "std_msgs/Float32") != 0)
        return;
      float rad = static_cast<const Float32Msg *>(sample)->data;
      float level = rad_to_level_(rad, sub.min_rad, sub.max_rad);
      sub.servo->write(level);
      this->remember_level_(sub.servo, level);
    }

    void Ros2Component::dispatch_joints_(const Subscription &sub, const void *sample)
    {
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
    }

    void Ros2Component::add_joint_subscription(const char *topic, const char *type, servo::Servo *servo,
                                               const char *joint_name, float min_rad, float max_rad)
    {
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
    }

    void Ros2Component::add_joint_state_source(servo::Servo *servo, const char *joint_name,
                                               float min_rad, float max_rad)
    {
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
    }

    uint8_t Ros2Component::add_image_publication(const char *topic, const char *type, camera::Camera *camera,
                                                uint32_t interval_ms)
    {
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
      if (strcmp(pub.type->name, "std_msgs/Float32") == 0)
      {
#ifdef USE_SENSOR
        if (pub.sensor == nullptr || !pub.sensor->has_state())
          return;
        Float32Msg msg;
        msg.data = pub.sensor->state;
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg));
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
          this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg));
      }
      else if (strcmp(pub.type->name, "sensor_msgs/JointState") == 0)
      {
        JointStateMsg msg;
        memset(&msg, 0, sizeof(msg));
        uint8_t n = pub.num_joints > ROS2_MAX_JOINTS ? ROS2_MAX_JOINTS : pub.num_joints;
        msg.num_joints = n;
        for (uint8_t i = 0; i < n; i++)
        {
          strncpy(msg.name[i], pub.joints[i].joint_name, ROS2_NAME_LEN - 1);
          float level = pub.joints[i].servo != nullptr ? this->recalled_level_(pub.joints[i].servo) : 0.0f;
          msg.position[i] = level_to_rad_(level, pub.joints[i].min_rad, pub.joints[i].max_rad);
        }
        this->mw_->publish(pub.topic, pub.type, &msg, sizeof(msg));
      }
    }

  } // namespace ros2
} // namespace esphome

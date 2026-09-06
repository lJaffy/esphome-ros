#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace esphome {
namespace ros2 {

struct TypeDef {
  const char *name;
  const char *dds_type_name;
  size_t size;
};

struct MiddlewareOptions {
  const char *key = nullptr;
  const char *value = nullptr;
};

using SampleCallback = std::function<void(const void *sample, size_t len)>;

class Ros2Middleware {
 public:
  virtual ~Ros2Middleware() = default;
  virtual bool subscribe(const std::string &topic, const TypeDef *type, SampleCallback cb,
                         const MiddlewareOptions *opts = nullptr) = 0;
  virtual bool unsubscribe(const std::string &topic) { return false; }
  virtual bool publish(const std::string &topic, const TypeDef *type, const void *sample, size_t len,
                       const MiddlewareOptions *opts = nullptr) = 0;
  // Publish a variable-length binary payload (e.g. JPEG frame) that has no
  // fixed-size sample struct. Default refuses; transports override as needed.
  virtual bool publish_image(const std::string &topic, const uint8_t *jpeg, size_t len) {
    (void) topic;
    (void) jpeg;
    (void) len;
    return false;
  }
  virtual bool connected() const = 0;
  virtual const char *name() const = 0;
};

class MiddlewareRegistry {
 public:
  static void register_middleware(const std::string &name, Ros2Middleware *mw);
  static Ros2Middleware *find(const std::string &name);
};

}  // namespace ros2
}  // namespace esphome

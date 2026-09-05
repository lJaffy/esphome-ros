#pragma once

#include <string>

#include "ros2_middleware.h"

namespace esphome {
namespace ros2 {

class JsonCodec {
 public:
  bool deserialize(const TypeDef *type, const std::string &payload, void *out, size_t out_len);
  std::string serialize(const TypeDef *type, const void *sample, size_t len);
};

}  // namespace ros2
}  // namespace esphome

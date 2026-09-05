#include "ros2_middleware.h"

#include <cstring>

namespace esphome {
namespace ros2 {

static const char *kNames[4] = {nullptr, nullptr, nullptr, nullptr};
static Ros2Middleware *kInstances[4] = {nullptr, nullptr, nullptr, nullptr};
static size_t kCount = 0;

void MiddlewareRegistry::register_middleware(const std::string &name, Ros2Middleware *mw) {
  for (size_t i = 0; i < kCount; i++) {
    if (name == kNames[i]) {
      kInstances[i] = mw;
      return;
    }
  }
  if (kCount < 4) {
    // Name storage: leak a copy on first registration; setup-time only.
    char *copy = new char[name.size() + 1];
    memcpy(copy, name.c_str(), name.size() + 1);
    kNames[kCount] = copy;
    kInstances[kCount] = mw;
    kCount++;
  }
}

Ros2Middleware *MiddlewareRegistry::find(const std::string &name) {
  for (size_t i = 0; i < kCount; i++) {
    if (name == kNames[i])
      return kInstances[i];
  }
  return nullptr;
}

}  // namespace ros2
}  // namespace esphome

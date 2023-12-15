#pragma once
#include <memory>
#include <functional>
#include <imgui.h>

namespace ImGui {
class Object {
public:
  using Ptr        = std::shared_ptr<Object>;
  using RenderImpl = std::function<void()>;

  Object() = delete;
  Object(RenderImpl impl)
    : mImpl{impl} {}
  ~Object() = default;

  void       render() { mImpl(); }
  RenderImpl mImpl;
};
}  //namespace ImGui
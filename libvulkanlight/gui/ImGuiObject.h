#pragma once
#include <memory>
#include <functional>
#include <imgui.h>

namespace ImGui {
class Object {
public:
  using Ptr        = std::shared_ptr<Object>;
  using RenderImpl = std::function<void()>;

  Object() {
    mImpl = []() {};
  }

  Object(RenderImpl impl)
    : mImpl{impl} {}
  ~Object() = default;

  void       render() { mImpl(); }
  void       setRenderImpl(RenderImpl impl) { mImpl = impl; }
  RenderImpl mImpl;
};
}  //namespace ImGui
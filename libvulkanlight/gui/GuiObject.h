#pragma once
#include <string>
#include <memory>
#include <functional>
#include <atomic>
#include <imgui.h>

namespace vkl {
class Button {
public:
  using Ptr = std::shared_ptr<Button>;
  Button(const std::string& n)
    : name(n)
    , mVirtualClick{false} {
    clickCB = []() {};
  }
  void setName(const std::string& n) { name = n; }
  bool isClicked() {
    if (mVirtualClick) {
      mVirtualClick = false;
      return true;
    }
    return ImGui::Button(name.c_str());
  }
  void registerCallback(std::function<void()> cb) { clickCB = cb; }
  void onClicked() { clickCB(); }
  void click() { mVirtualClick = true; }

protected:
  std::string           name = "";
  std::function<void()> clickCB;
  std::atomic<bool>     mVirtualClick = false;
};

class GuiImpl {
public:
  using Ptr        = std::shared_ptr<GuiImpl>;
  using RenderImpl = std::function<void()>;

  GuiImpl() {
    mImpl = []() {};
  }

  GuiImpl(RenderImpl impl)
    : mImpl{impl} {}
  ~GuiImpl() = default;

  void       render() { mImpl(); }
  void       setRenderImpl(RenderImpl impl) { mImpl = impl; }
  RenderImpl mImpl;
};
}  //namespace vkl
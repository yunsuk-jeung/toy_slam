#pragma once

#include <functional>
#include <memory>
#include "macros.h"

namespace vkl {
class InputCallback {
public:
  USING_SMART_PTR(InputCallback);

  InputCallback();
  virtual ~InputCallback();
  //Define the types for the various callbacks
  using MouseCB    = std::function<void(int i, int x, int y)>;
  using KeyBoardCB = std::function<void(int key)>;

  using MouseCBPtr    = std::unique_ptr<MouseCB>;
  using KeyBoardCBPtr = std::unique_ptr<KeyBoardCB>;

  void onMouseClick(int i, int x, int y);
  void onMouseDrag(int i, int x, int y);
  void onMouseWheel(int scroll);

  void onKeyPressed(int key);
  void onKeyDown(int key);
  void onKeyRelease(int key);

  void registerMouseClick(MouseCB&& cb);
  void registerMouseDrag(MouseCB&& cb);
  void registerMouseWheel(KeyBoardCB&& cb);

  void registerKeyPressed(KeyBoardCB&& cb);
  void registerKeyDown(KeyBoardCB&& cb);
  void registerKeyRelease(KeyBoardCB&& cb);

protected:
  MouseCBPtr    MouseClick;
  MouseCBPtr    MouseDrag;
  KeyBoardCBPtr MouseWheel;

  KeyBoardCBPtr KeyPressed;
  KeyBoardCBPtr KeyDown;
  KeyBoardCBPtr KeyRelease;

public:

protected:
};

}  //namespace vkl
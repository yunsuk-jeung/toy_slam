#include "InputCallback.h"
namespace vkl {
InputCallback::InputCallback()
  : MouseClick{nullptr}
  , MouseDrag{nullptr}
  , MouseWheel{nullptr}
  , KeyPressed{nullptr}
  , KeyDown{nullptr}
  , KeyRelease{nullptr} {}

InputCallback::~InputCallback() {}

void InputCallback::onMouseClick(int i, int x, int y) {
  if (MouseClick) (*MouseClick)(i, x, y);
}

void InputCallback::onMouseDrag(int i, int x, int y) {
  if (MouseDrag) (*MouseDrag)(i, x, y);
}
void InputCallback::onMouseWheel(int scroll) {
  if (MouseWheel) (*MouseWheel)(scroll);
}
void InputCallback::onKeyPressed(int key) {
  if (KeyPressed) (*KeyPressed)(key);
}

void InputCallback::onKeyDown(int key) {
  if (KeyDown) (*KeyDown)(key);
}

void InputCallback::onKeyRelease(int key) {
  if (KeyRelease) (*KeyRelease)(key);
}

void InputCallback::registerMouseClick(MouseCB&& cb) {
  MouseClick = std::make_unique<MouseCB>(cb);
}

void InputCallback::registerMouseDrag(MouseCB&& cb) {
  MouseDrag = std::make_unique<MouseCB>(cb);
}

void InputCallback::registerMouseWheel(KeyBoardCB&& cb) {
  MouseWheel = std::make_unique<KeyBoardCB>(cb);
}

void InputCallback::registerKeyPressed(KeyBoardCB&& cb) {
  KeyPressed = std::make_unique<KeyBoardCB>(cb);
}

void InputCallback::registerKeyDown(KeyBoardCB&& cb) {
  KeyDown = std::make_unique<KeyBoardCB>(cb);
}

void InputCallback::registerKeyRelease(KeyBoardCB&& cb) {
  KeyRelease = std::make_unique<KeyBoardCB>(cb);
}
}  //namespace vkl

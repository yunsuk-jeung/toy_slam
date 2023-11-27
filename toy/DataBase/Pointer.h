#pragma once

namespace toy {
namespace db {
template <typename T>
class Pointer {
public:
  Pointer()
    : mPointer{nullptr} {}
  Pointer(T* in) { mPointer = in; }
  virtual ~Pointer() = default;

  T*       operator->() { return mPointer; }
  const T* operator->() const { return mPointer; }

  T*           get() { return mPointer; }
  const T*     get() const { return mPointer; }
  virtual void release() = 0;

protected:
  T* mPointer;
};
}  //namespace db
}  //namespace toy
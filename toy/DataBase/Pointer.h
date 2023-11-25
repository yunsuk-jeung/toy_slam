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

  T*           get() { return mPointer; }
  const T*     get() const { return mPointer; }
  virtual void release() = 0;

protected:
  T* mPointer;
};
}  //namespace db
}  //namespace toy
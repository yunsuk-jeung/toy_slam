#pragma once
#include <vulkan/vulkan.hpp>
#include "macros.h"
namespace vkl {
template <typename T>
class Object {
public:
  Object()
    : mVkObject{VK_NULL_HANDLE} {}
  ~Object() { mVkObject = VK_NULL_HANDLE; }

  DELETE_COPY_CONSTRUCTORS(Object);
  void operator=(Object&&) = delete;

  T getVkObject() { return mVkObject; }

protected:
  T mVkObject;
};
}  //namespace vkl
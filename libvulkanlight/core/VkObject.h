#pragma once
#include "macros.h"
#include <vulkan/vulkan.hpp>

namespace vkl {
template <typename T>
class VkObject {
public:
  VkObject()
    : mVkObject{VK_NULL_HANDLE} {}
  ~VkObject() { mVkObject = VK_NULL_HANDLE; }

  DELETE_COPY_CONSTRUCTORS(VkObject);
  void operator=(VkObject&&) = delete;

  T& vk() { return mVkObject; }

protected:
  T mVkObject;
};
}  //namespace vkl
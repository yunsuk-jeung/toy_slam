#pragma once

#include <vulkan/vulkan.hpp>
#include "VkObject.h"
#include "macros.h"
namespace vkl {
class Device;
class Queue : public VkObject<vk::Queue> {
public:
  USING_SMART_PTR(Queue);
  DELETE_COPY_CONSTRUCTORS(Queue);
  Queue() = delete;
  Queue(Device*                    device,
        uint32_t                   family_index,
        vk::QueueFamilyProperties& props,
        vk::Bool32                 supportPresent,
        uint32_t                   index);

  explicit Queue(Queue&& other) noexcept;
  Queue& operator=(Queue&&) = delete;

protected:
  Device*                    device;
  uint32_t                   familyIdx;
  vk::QueueFamilyProperties* vkQueueFamilyProps;
  vk::Bool32                 supportPresent;
  uint32_t                   index;

public:
  vk::Queue&                 vk() { return mVkObject; }
  vk::QueueFamilyProperties* getVkQueueFamiliyProps() { return vkQueueFamilyProps; }

  const uint32_t             queueFamilyIdx() const { return familyIdx; }
  uint32_t                   getFamilyIdx() { return familyIdx; }
  vk::QueueFamilyProperties* getVkQueueFamilyProps() { return vkQueueFamilyProps; }
  vk::Bool32                 canPresent() { return supportPresent; }
};
}  //namespace vkl
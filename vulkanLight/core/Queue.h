#pragma once

#include <vulkan/vulkan.hpp>

namespace vkl {
class Device;
class Queue {

public:

  Queue() = delete;
  Queue(Device*                    device,
        uint32_t                   family_index,
        vk::QueueFamilyProperties& props,
        vk::Bool32                 supportPresent,
        uint32_t                   index);

protected:

  vk::Queue vkQueue{VK_NULL_HANDLE};

  Device*                    device{nullptr};
  uint32_t                   familyIdx{0};
  vk::QueueFamilyProperties* vkQueueFamilyProps{VK_NULL_HANDLE};
  vk::Bool32                 supportPresent{false};
  uint32_t                   index{0};

public:

  vk::Queue&                 getVkQueue() { return vkQueue; }
  vk::QueueFamilyProperties* getVkQueueFamiliyProps() { return vkQueueFamilyProps; }

  uint32_t                   getFamilyIdx() { return familyIdx; }
  vk::QueueFamilyProperties* getVkQueueFamilyProps() { return vkQueueFamilyProps; }
  vk::Bool32                 canPresent() { return supportPresent; }
};
}  //namespace vkl
#pragma once

#include <vector>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.h>
#include "core/Queue.h"
namespace vkl {

class Device {
public:
  Device() = delete;
  Device(vk::Instance instance, vk::PhysicalDevice device, vk::SurfaceKHR surface);
  ~Device();

  void initLogicalDevice();
  /**
   * @brief get queue that supports present orgrahics queue
   * @return queue that supports present or graphics queue
   */
  Queue& getPresentableQueue();
  Queue& getLowPrioritydQueue(vk::QueueFlags queueFlags);

  vk::Format getSuitableDepthFormat();
  uint32_t   findMemoryTypeIndex(const vk::MemoryRequirements& requirements,
                                 vk::MemoryPropertyFlags       properties);
  void       checkVkMemoryStatus();

protected:
  void     initVmaAllocator();
  uint32_t getQueueFamilyIndex(vk::QueueFlagBits queue_flag);
  Queue&   getQueueByFlags(vk::QueueFlags required_queue_flags, uint32_t queue_index);

protected:
  vk::Instance   vkInstance{VK_NULL_HANDLE};
  vk::SurfaceKHR vkSurface{VK_NULL_HANDLE};

  /**
   * @brief vsPhysicalDevice thigs
   */
  vk::PhysicalDevice         vkPhysicalDevice{VK_NULL_HANDLE};
  VkPhysicalDeviceProperties physicalDeviceProps;
  bool                       needHightPriorityGraphcisQueue{false};

  vk::PhysicalDeviceMemoryProperties memoryProps;
  VmaAllocator                       vmaAllocator{VK_NULL_HANDLE};

  std::vector<vk::QueueFamilyProperties> queueFamilyProps;
  std::vector<std::vector<Queue>>        queues;

  uint32_t graphicsQueueFamily;
  uint32_t computeQueueFamily;
  uint32_t transferQueueFamily;

  /**
   * @brief vkDevice thigs
   */
  vk::Device vkDevice{VK_NULL_HANDLE};

  vk::CommandPool renderCommandPool{VK_NULL_HANDLE};

public:
  //getters and setters
  vk::SurfaceKHR&     getVkSurface() { return vkSurface; };
  vk::PhysicalDevice& getVkPhysicalDevice() { return vkPhysicalDevice; }
  vk::Device&         getVkDevice() { return vkDevice; }
  VmaAllocator&       getMemoryAllocator() { return vmaAllocator; }
};
}  //namespace vkl
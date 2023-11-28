#include <Volk/volk.h>
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#include "Device.h"
#include "Queue.h"

#include "util/VkSettings.h"
#include "util/VkLogger.h"
#include "util/VkError.h"

namespace vkl {

Device::Device(vk::Instance instance, vk::PhysicalDevice device, vk::SurfaceKHR surface)
    : vkInstance(instance), vkPhysicalDevice(device), vkSurface(surface) {

  queueFamilyProps = vkPhysicalDevice.getQueueFamilyProperties();
  memoryProps      = vkPhysicalDevice.getMemoryProperties();
}

Device::~Device() {
  if (vmaAllocator != VK_NULL_HANDLE) {
    checkVkMemoryStatus();
    vmaDestroyAllocator(vmaAllocator);
  }

  vkDevice.destroy();
}

void Device::initLogicalDevice() {

  auto queueFamSize = queueFamilyProps.size();

  std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos(queueFamSize);
  std::vector<std::vector<float>>        queuePriorities(queueFamSize);

  for (auto queueFamIdx = 0U; queueFamIdx < queueFamSize; ++queueFamIdx) {
    auto& queueFamilyProp = queueFamilyProps[queueFamIdx];

    //if you need high priority graphics Queue,
    //set firt queue's priority bigger than others
    if (VkSettings::highPriorityGraphicsQueue) {
      auto graphicsQueueFamily = getQueueFamilyIndex(vk::QueueFlagBits::eGraphics);

      if (graphicsQueueFamily == queueFamIdx) {
        queuePriorities[queueFamIdx].resize(queueFamilyProp.queueCount, 0.5f);
        queuePriorities[queueFamIdx][0] = 1.0f;
      }
      else {
        queuePriorities[queueFamIdx].resize(queueFamilyProp.queueCount, 0.5f);
      }
    }
    else {
      queuePriorities[queueFamIdx].resize(queueFamilyProp.queueCount, 0.5f);
    }

    vk::DeviceQueueCreateInfo& queueCreateInfo = queueCreateInfos[queueFamIdx];

    queueCreateInfo.queueFamilyIndex = queueFamIdx;
    queueCreateInfo.queueCount       = queueFamilyProp.queueCount;
    queueCreateInfo.pQueuePriorities = queuePriorities[queueFamIdx].data();
  }

  std::vector<const char*>& extensions = VkSettings::enabledDeviceExtensions;

  vk::DeviceCreateInfo deviceCreateInfo{{}, queueCreateInfos, {}, extensions, {}};

  vkDevice = vkPhysicalDevice.createDevice(deviceCreateInfo);
  if (!vkDevice) {
    LOGE("Could not create vkDevice!");
    throw std::runtime_error("Could not create vkDevice");
  }

  //save every queue data
  queues.resize(queueFamSize);
  for (auto queueFamIdx = 0U; queueFamIdx < queueFamSize; ++queueFamIdx) {

    vk::QueueFamilyProperties& queueFamilyprop = queueFamilyProps[queueFamIdx];

    vk::Bool32 supportPresent
        = vkPhysicalDevice.getSurfaceSupportKHR(queueFamIdx, vkSurface);

    for (auto queueIndex = 0U; queueIndex < queueFamilyprop.queueCount; ++queueIndex) {

      queues[queueFamIdx].emplace_back(this,
                                       queueFamIdx,
                                       queueFamilyprop,
                                       supportPresent,
                                       queueIndex);
    }
  }

  //Queue& primaryQueue
  //    = getQueue(vk::QueueFlagBits::eGraphics | vk::QueueFlagBits::eCompute,
  //    0);

  //vk::CommandPoolCreateInfo poolInfo{
  //    vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
  //    primaryQueue.getFamilyIdx()};

  //if (!vkPhysicalDevice.getSurfaceSupportKHR(primaryQueue.getFamilyIdx(),
  //                                           vkSurface)) {
  //  LOGW("This queue does not supprot SurfaceKHR");
  //  throw std::runtime_error("This queue does not supprot SurfaceKHR");
  //}

  //renderCommandPool = vkDevice.createCommandPool(poolInfo);

  initVmaAllocator();
}

Queue& Device::getPresentableQueue() {
  for (auto queueFamIdx = 0U; queueFamIdx < queues.size(); ++queueFamIdx) {

    Queue&   frontQueue = queues[queueFamIdx].front();
    uint32_t queueCount = frontQueue.getVkQueueFamilyProps()->queueCount;

    if (frontQueue.canPresent() && queueCount > 0) {
      return queues[queueFamIdx][0];
    }
  }

  return getQueueByFlags(vk::QueueFlagBits::eGraphics, 0);
}

vk::Format Device::getSuitableDepthFormat() {
  vk::Format depthFormat{vk::Format::eUndefined};

  const auto& priorities = VkSettings::depthFormatPriorities;

  for (auto& format : priorities) {
    vk::FormatProperties props = vkPhysicalDevice.getFormatProperties(format);

    if (props.optimalTilingFeatures
        & vk::FormatFeatureFlagBits::eDepthStencilAttachment) {
      depthFormat = format;
      break;
    }
  }

  //if (depthFormat != vk::Format::eUndefined) {
  //  LOGI("Depth Format : {}", to_string(depthFormat).c_str());
  //}
  return depthFormat;
}

uint32_t Device::findMemoryTypeIndex(const vk::MemoryRequirements& requirements,
                                     vk::MemoryPropertyFlags       properties) {

  for (auto i = 0; i != memoryProps.memoryTypeCount; ++i) {
    if (!(requirements.memoryTypeBits & (1 << i))) continue;

    if ((memoryProps.memoryTypes[i].propertyFlags & properties) != properties) continue;

    return i;
  }

  assert(false);
  return UINT32_MAX;
}

void Device::checkVkMemoryStatus() {
  VmaTotalStatistics stats;
  vmaCalculateStatistics(vmaAllocator, &stats);

  //LOGE("Total device blockCount     : {} bytes.", stats.total.statistics.blockCount);
  //LOGE("Total device allocationCount: {} bytes.",
  //stats.total.statistics.allocationCount); LOGE("Total device blockBytes     : {}
  //bytes.", stats.total.statistics.blockBytes);
  LOGW("Total device allocationBytes: {} bytes.", stats.total.statistics.allocationBytes);
  //LOGW("----------------------------------------");
}

void Device::initVmaAllocator() {
  VmaVulkanFunctions vmaVkFunc{};
  vmaVkFunc.vkGetInstanceProcAddr               = vkGetInstanceProcAddr;
  vmaVkFunc.vkGetDeviceProcAddr                 = vkGetDeviceProcAddr;
  vmaVkFunc.vkAllocateMemory                    = vkAllocateMemory;
  vmaVkFunc.vkBindBufferMemory                  = vkBindBufferMemory;
  vmaVkFunc.vkBindImageMemory                   = vkBindImageMemory;
  vmaVkFunc.vkCreateBuffer                      = vkCreateBuffer;
  vmaVkFunc.vkCreateImage                       = vkCreateImage;
  vmaVkFunc.vkDestroyBuffer                     = vkDestroyBuffer;
  vmaVkFunc.vkDestroyImage                      = vkDestroyImage;
  vmaVkFunc.vkFlushMappedMemoryRanges           = vkFlushMappedMemoryRanges;
  vmaVkFunc.vkFreeMemory                        = vkFreeMemory;
  vmaVkFunc.vkGetBufferMemoryRequirements       = vkGetBufferMemoryRequirements;
  vmaVkFunc.vkGetImageMemoryRequirements        = vkGetImageMemoryRequirements;
  vmaVkFunc.vkGetPhysicalDeviceMemoryProperties = vkGetPhysicalDeviceMemoryProperties;
  vmaVkFunc.vkGetPhysicalDeviceProperties       = vkGetPhysicalDeviceProperties;
  vmaVkFunc.vkInvalidateMappedMemoryRanges      = vkInvalidateMappedMemoryRanges;
  vmaVkFunc.vkMapMemory                         = vkMapMemory;
  vmaVkFunc.vkUnmapMemory                       = vkUnmapMemory;
  vmaVkFunc.vkCmdCopyBuffer                     = vkCmdCopyBuffer;

  VmaAllocatorCreateInfo allocator_info{};
  allocator_info.physicalDevice = static_cast<VkPhysicalDevice>(vkPhysicalDevice);
  allocator_info.device         = static_cast<VkDevice>(vkDevice);
  allocator_info.instance       = static_cast<VkInstance>(vkInstance);

  bool canGetMemroy
      = VkSettings::isInDeviceExtension(VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME);

  bool hasDedicatedAlloc
      = VkSettings::isInDeviceExtension(VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME);

  if (canGetMemroy && hasDedicatedAlloc) {
    allocator_info.flags |= VMA_ALLOCATOR_CREATE_KHR_DEDICATED_ALLOCATION_BIT;
    vmaVkFunc.vkGetBufferMemoryRequirements2KHR = vkGetBufferMemoryRequirements2KHR;
    vmaVkFunc.vkGetImageMemoryRequirements2KHR  = vkGetImageMemoryRequirements2KHR;
  }
  bool hasBufferDeviceAddress
      = VkSettings::isInDeviceExtension(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);

  if (hasBufferDeviceAddress) {
    allocator_info.flags |= VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
  }

  allocator_info.pVulkanFunctions = &vmaVkFunc;
  VK_CHECK_ERROR(vmaCreateAllocator(&allocator_info, &vmaAllocator),
                 "Cannot create allocator");
}

uint32_t Device::getQueueFamilyIndex(vk::QueueFlagBits queueFlags) {
  //Dedicated queue for compute
  //Try to find a queue family index that supports compute but not graphics
  auto queueSize = queueFamilyProps.size();

  if ((queueFlags & vk::QueueFlagBits::eCompute) == queueFlags) {
    for (uint32_t i = 0; i < queueSize; i++) {
      if ((queueFamilyProps[i].queueFlags & vk::QueueFlagBits::eCompute)
          && !(queueFamilyProps[i].queueFlags & vk::QueueFlagBits::eGraphics)) {
        return i;
      }
    }
  }

  //Dedicated queue for transfer
  //Try to find a queue family index that supports transfer but not graphics
  //and compute

  if ((queueFlags & vk::QueueFlagBits::eTransfer) == queueFlags) {
    for (uint32_t i = 0; i < queueSize; i++) {
      if ((queueFamilyProps[i].queueFlags & vk::QueueFlagBits::eTransfer)
          && !(queueFamilyProps[i].queueFlags & vk::QueueFlagBits::eGraphics)
          && !(queueFamilyProps[i].queueFlags & vk::QueueFlagBits::eCompute)) {
        return i;
      }
    }
  }

  //For other queue types or if no separate compute queue is present, return the
  //first one to support the requested flags
  for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProps.size()); i++) {
    if ((queueFamilyProps[i].queueFlags & queueFlags) == queueFlags) {
      return i;
    }
  }

  LOGW("Cannot find queue Family for {}", static_cast<int>(queueFlags));
  throw std::runtime_error("Could not find a matching queue family index");
}

Queue& Device::getQueueByFlags(vk::QueueFlags requiredFlags, uint32_t queueidx) {

  auto queueFamSize = queues.size();

  for (auto queueFamIdx = 0U; queueFamIdx < queueFamSize; ++queueFamIdx) {

    auto& queue = queues[queueFamIdx][0];

    vk::QueueFlags queue_flags = queue.getVkQueueFamiliyProps()->queueFlags;
    uint32_t       queue_count = queue.getVkQueueFamiliyProps()->queueCount;

    if (((queue_flags & requiredFlags) == requiredFlags) && queueidx < queue_count) {
      return queues[queueFamIdx][queueidx];
    }
  }

  throw std::runtime_error("Queue not found");
}

}  //namespace vkl
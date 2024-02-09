#include "Queue.h"
#include "Device.h"

namespace vkl {
Queue::Queue(Device*                    _device,
             uint32_t                   _familyIdx,
             vk::QueueFamilyProperties& props,
             vk::Bool32                 supportPresent,
             uint32_t                   index)
  : device{_device}
  , familyIdx{_familyIdx}
  , vkQueueFamilyProps{&props}
  , supportPresent{supportPresent}
  , index{index} {
  mVkObject = device->vk().getQueue(familyIdx, index);
}

Queue::Queue(Queue&& src) noexcept
  : device{src.device}
  , familyIdx{src.familyIdx}
  , vkQueueFamilyProps{src.vkQueueFamilyProps}
  , supportPresent{src.supportPresent}
  , index{src.index} {
  mVkObject              = src.mVkObject;
  src.mVkObject          = nullptr;
  src.device             = nullptr;
  src.familyIdx          = 0;
  src.vkQueueFamilyProps = nullptr;
  src.supportPresent     = false;
  src.index              = 0;
}
}  //namespace vkl

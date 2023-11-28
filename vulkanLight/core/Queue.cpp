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

  vkQueue = device->getVkDevice().getQueue(familyIdx, index);
}
}  //namespace vkl

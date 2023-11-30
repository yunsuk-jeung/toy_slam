#pragma once
#include "RendererBase.h"

namespace vkl {
class BasicRenderer : public RendererBase{
  BasicRenderer() = delete;
  BasicRenderer(Device* device, RenderContext* context, vk::DescriptorPool descPool);

  virtual ~BasicRenderer();
};
}
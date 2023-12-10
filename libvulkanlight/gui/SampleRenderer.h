#pragma once
#include "RendererBase.h"

namespace vkl {
class Buffer;
class BufferingBuffer;
class SampleRenderer : public RendererBase {
public:
  SampleRenderer();
  virtual ~SampleRenderer();

  void buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx);

protected:
  virtual void createVertexBuffer() override;

  std::unique_ptr<Buffer>          mVB;
  std::unique_ptr<BufferingBuffer> mBVB;
};
}  //namespace vkl
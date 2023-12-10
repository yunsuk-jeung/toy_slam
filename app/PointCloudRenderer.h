#pragma once
#include <vector>
#include "RendererBase.h"

namespace vkl {
class BufferingBuffer;
class UniformBuffer;
class PointCloudRenderer : public RendererBase {
public:
  PointCloudRenderer();
  ~PointCloudRenderer();

  virtual void createVertexBuffer() override;
  virtual void createUniformBuffers() override;

  void buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx);

protected:
  std::vector<float>               mPointBuffer;
  std::unique_ptr<BufferingBuffer> mBVB;
  std::unique_ptr<UniformBuffer>   mUB;
};
}  //namespace vkl
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
  void         updateSyncId();
  void         buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx);

  void setPointCloud(std::vector<float>* pointBuffer) { mPointBufferPtr = pointBuffer; }

protected:
  std::vector<float>*              mPointBufferPtr;
  uint32_t                         mSyncId;
  std::vector<uint32_t>            mSyncIds;
  std::unique_ptr<BufferingBuffer> mBVB;
  std::unique_ptr<UniformBuffer>   mUB;
};
}  //namespace vkl
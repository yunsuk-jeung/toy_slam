#pragma once
#include <vector>
#include "RendererBase.h"

namespace vkl {
class BufferingBuffer;
class UniformBuffer;
class DescriptorSetLayout;
class PointCloudRenderer : public RendererBase {
public:
  PointCloudRenderer();
  ~PointCloudRenderer();

  virtual void createVertexBuffer() override;
  virtual void createUniformBuffer() override;
  void         updateSyncId();
  void         buildCommandBuffer(vk::CommandBuffer cmd,
                                  uint32_t          idx,
                                  vk::DescriptorSet camDescSet);

  void setPointCloud(std::vector<float>* pointBuffer) { mPointBufferPtr = pointBuffer; }

protected:
  std::vector<float>*              mPointBufferPtr;
  uint32_t                         mSyncId;
  std::vector<uint32_t>            mSyncIds;
  std::unique_ptr<BufferingBuffer> mBVB;

  struct ModelDescriptorSet {
    DescriptorSetLayout*           descLayout;
    std::vector<vk::DescriptorSet> descSets;
    std::unique_ptr<UniformBuffer> mUB;
  } mModelDescriptorSet;
};
}  //namespace vkl
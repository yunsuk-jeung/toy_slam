#pragma once
#include <Eigen/Dense>
#include "RendererBase.h"
namespace vkl {
class Buffer;
class DynamicUniformBuffer;
class DescriptorSetLayout;
class AxisRenderer : public RendererBase {
public:
  AxisRenderer();
  AxisRenderer(int maxAxisSize);
  ~AxisRenderer();

  void setMwcs(std::vector<Eigen::Matrix4f>* pMwcs) { mMwcsPtr = pMwcs; }
  void updateSyncId();
  void buildCommandBuffer(vk::CommandBuffer cmd,
                          uint32_t          idx,
                          vk::DescriptorSet camDescset);

protected:
  virtual void createVertexBuffer() override;
  virtual void createUniformBuffer() override;

protected:
  uint32_t                      mSyncId;
  std::vector<uint32_t>         mSyncIds;
  std::unique_ptr<Buffer>       mVB;
  std::vector<Eigen::Matrix4f>* mMwcsPtr;

  struct ModelDesciptor {
    DescriptorSetLayout*                  descLayout;
    std::vector<vk::DescriptorSet>        descSets;
    std::unique_ptr<DynamicUniformBuffer> DUB;
  } mModelDesciptor;

  const int mMaxAixsSize;
};
}  //namespace vkl
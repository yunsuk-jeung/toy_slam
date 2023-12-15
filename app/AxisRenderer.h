#pragma once
#include <Eigen/Dense>
#include "RendererBase.h"
namespace vkl {
class Buffer;
class DynamicUniformBuffer;

class AxisRenderer : public RendererBase {
public:
  AxisRenderer();
  AxisRenderer(int maxAxisSize);
  ~AxisRenderer();

  void setMwcs(std::vector<Eigen::Matrix4f>* pMwcs) { mMwcsPtr = pMwcs; }
  void updateSyndId();
  void buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx);

protected:
  virtual void createVertexBuffer() override;
  virtual void createUniformBuffers() override;

protected:
  uint32_t                              mSyncId;
  std::vector<uint32_t>                 mSyncIds;
  std::unique_ptr<Buffer>               mVB;
  std::unique_ptr<DynamicUniformBuffer> mDUB;
  std::vector<Eigen::Matrix4f>*         mMwcsPtr;

  const int mMaxAixsSize;
};
}  //namespace vkl
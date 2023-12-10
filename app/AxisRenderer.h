#pragma once
#include <Eigen/Dense>
#include "RendererBase.h"
namespace vkl {
class Buffer;
class DynamicUniformBuffer;

class AxisRenderer : public RendererBase {
public:
  AxisRenderer();
  ~AxisRenderer();

  void buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx);

protected:
  virtual void createVertexBuffer() override;
  virtual void createUniformBuffers() override;

  std::unique_ptr<Buffer>               mVB;
  std::unique_ptr<DynamicUniformBuffer> mDUB;
  std::vector<Eigen::Matrix4f>          mMs;
};
}  //namespace vkl
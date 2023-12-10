#include <vector>
#include "ShaderTypes.h"
#include "UniformBuffer.h"
#include "RenderContext.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "AxisRenderer.h"

namespace vkl {
AxisRenderer::AxisRenderer() {
  mName = "Axis Renderer";
}

AxisRenderer::~AxisRenderer() {}

void AxisRenderer::buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx) {
  auto& vkBuffer     = mVB->vk();
  auto& camDescSet   = mCamUB->getVkDescSet(idx);
  auto& modelDescSet = mUB->getVkDescSet(idx);

  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, mPipeline->vk());
  cmd.bindVertexBuffers(0, {vkBuffer}, {0});
  cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                         mPipelineLayout->vk(),
                         0,
                         {camDescSet, modelDescSet},
                         {});

  cmd.draw(6, 1, 0, 0);
}

void AxisRenderer::createVertexBuffer() {
  std::vector<VertexColor> vertices = {
    {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f}
  };

  auto memSize = sizeof(VertexColor) * vertices.size();
  mVB          = std::make_unique<Buffer>(mDevice,
                                 memSize,
                                 vk::BufferUsageFlagBits::eVertexBuffer,
                                 vk::MemoryPropertyFlagBits::eHostVisible,
                                 vk::MemoryPropertyFlagBits::eHostCoherent);

  mVB->update(vertices.data(), memSize, 0);
}

void AxisRenderer::createUniformBuffers() {
  auto count         = mRenderContext->getContextImageCount();
  auto size          = sizeof(Eigen::Matrix4f);
  auto descsetLayout = mPipelineLayout->getDescriptorSetLayout("modelMat");

  mUB = std::make_unique<UniformBuffer>(mDevice,
                                        mVkDescPool,
                                        descsetLayout,
                                        0,
                                        count,
                                        size);

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();

  for (int i = 0; i < count; ++i) { mUB->update(i, I.data()); }
}
}  //namespace vkl
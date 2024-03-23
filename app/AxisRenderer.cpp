#include <vector>
#include "VklLogger.h"
#include "ShaderTypes.h"
#include "UniformBuffer.h"
#include "DynamicUniformBuffer.h"
#include "RenderContext.h"
#include "PipelineLayout.h"
#include "GraphicsPipeline.h"
#include "AxisRenderer.h"

namespace vkl {

AxisRenderer::AxisRenderer()
  : mMaxAixsSize{2000}
  , mSyncId{0}
  , mSyncIds{}
  , mModelDesciptor{nullptr, {}, nullptr} {
  mName = "Axis Renderer";
}

AxisRenderer::AxisRenderer(int maxAxisSize)
  : mMaxAixsSize{maxAxisSize} {
  mName = "Axis Renderer";
}

AxisRenderer::~AxisRenderer() {}

void AxisRenderer::updateSyncId() {
  ++mSyncId;
}

void AxisRenderer::buildCommandBuffer(vk::CommandBuffer cmd,
                                      uint32_t          idx,
                                      vk::DescriptorSet camDescSet) {
  if (mMwcsPtr->empty())
    return;
  auto size = mMwcsPtr->size();

  if (mSyncIds[idx] != mSyncId) {
    mModelDesciptor.DUB->update(idx, size, sizeof(Eigen::Matrix4f), mMwcsPtr->data());
    mSyncIds[idx] = mSyncId;
  }

  auto& vkBuffer     = mVB->vk();
  auto& modelDescSet = mModelDesciptor.DUB->getVkDescSet(idx);

  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, mPipeline->vk());
  cmd.bindVertexBuffers(0, {vkBuffer}, {0});
  cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                         mPipelineLayout->vk(),
                         0,
                         {camDescSet},
                         {});

  for (int i = 0; i < size; i++) {
    uint32_t offset = i * mModelDesciptor.DUB->getAlignment();
    cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                           mPipelineLayout->vk(),
                           1,
                           {modelDescSet},
                           {offset});
    cmd.draw(6, 1, 0, 0);
  }
}

void AxisRenderer::createVertexBuffer() {
  std::vector<VertexColor> vertices = {
    { 0.0f,  0.0f,  0.0f, 1.0f, 0.0f, 0.0f},
    {0.25f,  0.0f,  0.0f, 1.0f, 0.0f, 0.0f},
    { 0.0f,  0.0f,  0.0f, 0.0f, 1.0f, 0.0f},
    { 0.0f, 0.25f,  0.0f, 0.0f, 1.0f, 0.0f},
    { 0.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f},
    { 0.0f,  0.0f, 0.25f, 0.0f, 0.0f, 1.0f}
  };

  auto memSize = sizeof(VertexColor) * vertices.size();
  mVB          = std::make_unique<Buffer>(mDevice,
                                 memSize,
                                 vk::BufferUsageFlagBits::eVertexBuffer,
                                 vk::MemoryPropertyFlagBits::eHostVisible,
                                 vk::MemoryPropertyFlagBits::eHostCoherent);

  mVB->update(vertices.data(), memSize, 0);
}

void AxisRenderer::createUniformBuffer() {
  auto count = mRenderContext->getContextImageCount();
  mSyncIds.resize(count, 0);

  auto minVkMemoryAlignment = mDevice->getVkPhysicalDevice()
                                .getProperties()
                                .limits.minUniformBufferOffsetAlignment;

  uint32_t uniformSize = sizeof(Eigen::Matrix4f);
  uint32_t alignment   = uniformSize;

  if (alignment % minVkMemoryAlignment != 0) {
    alignment = minVkMemoryAlignment;

    if (uniformSize > minVkMemoryAlignment)
      vklLogE("you have to implement this part");
  }
  mModelDesciptor.descLayout = mPipelineLayout->getDescriptorSetLayout(1u);

  mModelDesciptor.descSets.resize(count);
  for (size_t i = 0u; i < count; ++i) {
    mModelDesciptor.descSets[i] = mDevice->vk()
                                    .allocateDescriptorSets(
                                      {mVkDescPool, mModelDesciptor.descLayout->vk()})
                                    .front();
  }

  auto memSize        = mMaxAixsSize * alignment;
  mModelDesciptor.DUB = std::make_unique<DynamicUniformBuffer>(mDevice,
                                                               mModelDesciptor.descSets,
                                                               0u,
                                                               memSize,
                                                               uniformSize,
                                                               alignment);
}
}  //namespace vkl
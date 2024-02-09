#include "VklLogger.h"
#include "RenderContext.h"
#include "BufferingBuffer.h"
#include "UniformBuffer.h"
#include "PipelineLayout.h"
#include "GraphicsPipeline.h"
#include "PointCloudRenderer.h"

#include <random>
#include <cmath>

namespace vkl {
namespace {
constexpr size_t initialSize = 4 * 100000;
}
PointCloudRenderer::PointCloudRenderer()
  : mBVB{nullptr}
  , mModelDescriptorSet{nullptr, {}, nullptr}
  , mSyncId{0} {
  mName = "PointCloud Renderer";
}

PointCloudRenderer::~PointCloudRenderer() {
  mBVB.reset();
}

void PointCloudRenderer::createVertexBuffer() {
  auto count = mRenderContext->getContextImageCount();
  mSyncIds.resize(count, 0);
  mBVB = std::make_unique<BufferingBuffer>(mDevice,
                                           count,
                                           initialSize,
                                           vk::BufferUsageFlagBits::eVertexBuffer,
                                           vk::MemoryPropertyFlagBits::eHostVisible,
                                           vk::MemoryPropertyFlagBits::eHostCoherent);

  //constexpr float                  radius = 3.0f;
  //std::random_device               rd;
  //std::mt19937                     gen(rd());
  //std::uniform_real_distribution<> dis(-radius, radius);

  //for (int i = 0; i < 30000; ++i) {
  //  float x, y, z;
  //  do {
  //    x = dis(gen);
  //    y = dis(gen);
  //    z = dis(gen);
  //  } while (std::sqrt(x * x + y * y + z * z) > radius);

  //mPointBuffer.push_back(x);
  //mPointBuffer.push_back(y);
  //mPointBuffer.push_back(z);
  //mPointBuffer.push_back(1.0f);
  //}

  //auto memorySize = sizeof(float) * mPointBuffer.size();

  //for (int i = 0; i < 3; i++) { mBVB->update(i, mPointBuffer.data(), memorySize, 0); }
}

void PointCloudRenderer::createUniformBuffer() {
  auto count                     = mRenderContext->getContextImageCount();
  auto size                      = sizeof(Eigen::Matrix4f);
  mModelDescriptorSet.descLayout = mPipelineLayout->getDescriptorSetLayout(1u);

  mModelDescriptorSet.descSets.resize(count);
  for (size_t i = 0u; i < count; ++i) {
    mModelDescriptorSet.descSets[i] = mDevice->vk()
                                        .allocateDescriptorSets(
                                          {mVkDescPool,
                                           mModelDescriptorSet.descLayout->vk()})
                                        .front();
  }

  mModelDescriptorSet.mUB = std::make_unique<UniformBuffer>(mDevice,
                                                            mModelDescriptorSet.descSets,
                                                            0,
                                                            size);

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();

  for (int i = 0; i < count; ++i) {
    mModelDescriptorSet.mUB->update(i, I.data());
  }
}

void PointCloudRenderer::updateSyncId() {
  ++mSyncId;
}

void PointCloudRenderer::buildCommandBuffer(vk::CommandBuffer cmd,
                                            uint32_t          idx,
                                            vk::DescriptorSet camDescSet) {
  if (mPointBufferPtr->empty())
    return;

  if (mSyncIds[idx] != mSyncId) {
    auto memorySize = sizeof(float) * mPointBufferPtr->size();
    mBVB->update(idx, mPointBufferPtr->data(), memorySize, 0);
    mSyncIds[idx] = mSyncId;
  }

  auto& vkBuffer     = mBVB->getVkBuffer(idx);
  auto& modelDescSet = mModelDescriptorSet.descSets[idx];
  auto  vertexCount  = mPointBufferPtr->size() >> 2;

  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, mPipeline->vk());
  cmd.bindVertexBuffers(0, {vkBuffer}, {0});
  cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                         mPipelineLayout->vk(),
                         0,
                         {camDescSet, modelDescSet},
                         {});

  cmd.draw(vertexCount, 1, 0, 0);
}

}  //namespace vkl
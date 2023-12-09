#include "VklLogger.h"
#include "RenderContext.h"
#include "BufferingBuffer.h"
#include "UniformBuffer.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "PointCloudRenderer.h"

#include <random>
#include <cmath>

namespace vkl {
namespace {
constexpr size_t initialSize = 4 * 100000;

}
PointCloudRenderer::PointCloudRenderer()
  : mBVB{nullptr}
  , mUB{nullptr} {
  mPointBuffer.reserve(initialSize);
}

PointCloudRenderer::~PointCloudRenderer() {
  mUB.reset();
  mBVB.reset();
}

void PointCloudRenderer::createVertexBuffer() {
  auto count = mRenderContext->getContextImageCount();

  mBVB = std::make_unique<BufferingBuffer>(mDevice,
                                           count,
                                           initialSize,
                                           vk::BufferUsageFlagBits::eVertexBuffer,
                                           vk::MemoryPropertyFlagBits::eHostVisible,
                                           vk::MemoryPropertyFlagBits::eHostCoherent);

  constexpr float                  radius = 1.0f;
  std::random_device               rd;
  std::mt19937                     gen(rd());
  std::uniform_real_distribution<> dis(-radius, radius);

  for (int i = 0; i < 1000; ++i) {
    float x, y, z;
    do {
      x = dis(gen);
      y = dis(gen);
      z = dis(gen);
    } while (std::sqrt(x * x + y * y + z * z) > radius);

    mPointBuffer.push_back(x);
    mPointBuffer.push_back(y);
    mPointBuffer.push_back(z);
    mPointBuffer.push_back(1.0f);
  }

  auto memorySize = sizeof(float) * mPointBuffer.size();

  for (int i = 0; i < 3; i++) { mBVB->update(i, mPointBuffer.data(), memorySize, 0); }
}

void PointCloudRenderer::createUniformBuffers() {
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

  mUB->update(0, I.data());
  mUB->update(1, I.data());
  mUB->update(2, I.data());
}

void PointCloudRenderer::buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx) {
  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, mPipeline->vk());

  mBVB->at(idx)->vk();

  cmd.bindVertexBuffers(0, {mBVB->at(idx)->vk()}, {0});

  auto& camDescSet = mCamUB->getVkDescSet(idx);
  auto& modelSet   = mUB->getVkDescSet(idx);

  cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                         mPipelineLayout->vk(),
                         0,
                         {camDescSet, modelSet},
                         {});

  auto vertexCount = mPointBuffer.size() >> 2;

  cmd.draw(vertexCount, 1, 0, 0);
}

void PointCloudRenderer::setName() {
  mName = "PointCloud Renderer";
}

}  //namespace vkl
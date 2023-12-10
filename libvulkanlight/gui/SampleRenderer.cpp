#pragma once
#include "VklLogger.h"
#include "Device.h"
#include "RenderContext.h"
#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "ShaderTypes.h"
#include "Buffer.h"
#include "BufferingBuffer.h"
#include "UniformBuffer.h"
#include "SampleRenderer.h"

namespace vkl {
SampleRenderer::SampleRenderer()
  : RendererBase() {
  mName = "SampleRenderer";
}

SampleRenderer::~SampleRenderer() {}

void SampleRenderer::buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx) {
  auto& camDescSet = mCamUB->getVkDescSet(idx);

  cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                         mPipelineLayout->vk(),
                         0,
                         {camDescSet},
                         {});

  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, mPipeline->vk());

  auto& vkBuffer = mBVB->at(idx)->vk();

  cmd.bindVertexBuffers(0, {vkBuffer}, {0});
  cmd.draw(3, 1, 0, 0);
}

void SampleRenderer::createVertexBuffer() {
  std::vector<Vertex4d> vertices = {
    {-0.0, -0.5, 0.2, 1.0},
    { 0.5,  0.5, 0.2, 1.0},
    {-0.5,  0.5, 0.2, 1.0}
  };

  mVB = std::make_unique<Buffer>(mDevice,
                                 sizeof(Vertex) * vertices.size(),
                                 vk::BufferUsageFlagBits::eVertexBuffer,
                                 vk::MemoryPropertyFlagBits::eHostVisible,
                                 vk::MemoryPropertyFlagBits::eHostCoherent);

  auto memorySize = sizeof(Vertex4d) * vertices.size();
  mVB->update(vertices.data(), memorySize, 0);

  mBVB = std::make_unique<BufferingBuffer>(mDevice,
                                           3,
                                           sizeof(Vertex) * vertices.size(),
                                           vk::BufferUsageFlagBits::eVertexBuffer,
                                           vk::MemoryPropertyFlagBits::eHostVisible,
                                           vk::MemoryPropertyFlagBits::eHostCoherent);
  mBVB->update(0, vertices.data(), memorySize, 0);
  mBVB->update(1, vertices.data(), memorySize, 0);
  mBVB->update(2, vertices.data(), memorySize, 0);
}
}  //namespace vkl
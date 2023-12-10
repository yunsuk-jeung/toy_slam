#pragma once
#include "VkError.h"
#include "VklLogger.h"
#include "Device.h"
#include "RenderContext.h"
#include "ResourcePool.h"
#include "ShaderModule.h"
#include "ShaderTypes.h"
#include "PipelineLayout.h"
#include "Pipeline.h"

namespace vkl {
class BasicPointPipeline : public Pipeline {
public:
  BasicPointPipeline(const std::string& name,
                     Device*            device,
                     RenderContext*     context,
                     vk::RenderPass     renderPass,
                     PipelineLayout*    pipelineLayout,
                     uint32_t           subpassId = 0)
    : Pipeline(name, device, context, renderPass, pipelineLayout, subpassId) {
    prepare();
  }

  ~BasicPointPipeline() = default;

  void prepare() override {
    auto& shaders = mPipelineLayout->getShaderModules();

    std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
      {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
      {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
    };

    std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
      {0, sizeof(Vertex4d), vk::VertexInputRate::eVertex}
    };

    std::vector<vk::VertexInputAttributeDescription> attributeDescription{
      {0, 0, vk::Format::eR32G32B32A32Sfloat, 0},
    };

    vk::PipelineVertexInputStateCreateInfo inputStateCI({},
                                                        vertBindingDescription,
                                                        attributeDescription);

    vk::PipelineInputAssemblyStateCreateInfo
      inputAssemCI({}, vk::PrimitiveTopology::ePointList, false);

    vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);

    vk::PipelineRasterizationStateCreateInfo rasterizationState;
    rasterizationState.polygonMode = vk::PolygonMode::eFill;
    rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
    rasterizationState.frontFace   = vk::FrontFace::eClockwise;
    rasterizationState.lineWidth   = 1.0f;

    vk::PipelineMultisampleStateCreateInfo multisample_state({},
                                                             vk::SampleCountFlagBits::e1);
    vk::PipelineColorBlendAttachmentState  blendAttachmentState;
    blendAttachmentState.blendEnable         = true;
    blendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
    blendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
    blendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
    blendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
    blendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
    blendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
    blendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
                                          | vk::ColorComponentFlagBits::eG
                                          | vk::ColorComponentFlagBits::eB
                                          | vk::ColorComponentFlagBits::eA;

    vk::PipelineDepthStencilStateCreateInfo depthStencilState;
    depthStencilState.depthCompareOp   = vk::CompareOp::eAlways;
    depthStencilState.depthTestEnable  = true;
    depthStencilState.depthWriteEnable = true;
    depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
    depthStencilState.front            = depthStencilState.back;

    vk::PipelineColorBlendStateCreateInfo colorBlendState({},
                                                          {},
                                                          {},
                                                          blendAttachmentState);

    std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
                                                           vk::DynamicState::eScissor};

    vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);

    vk::GraphicsPipelineCreateInfo pipelineCI({},
                                              shaderStageCIs,
                                              &inputStateCI,
                                              &inputAssemCI,
                                              {},
                                              &viewport_state,
                                              &rasterizationState,
                                              &multisample_state,
                                              &depthStencilState,
                                              &colorBlendState,
                                              &dynamicState,
                                              mPipelineLayout->vk(),
                                              mVkRenderPass,
                                              mSubpassId);

    vk::Result result;

    std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
                                                                       pipelineCI);
    VK_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
  }
};

class BasicLinePipeline : public Pipeline {
public:
  BasicLinePipeline(const std::string& name,
                    Device*            device,
                    RenderContext*     context,
                    vk::RenderPass     renderPass,
                    PipelineLayout*    pipelineLayout,
                    uint32_t           subpassId = 0)
    : Pipeline(name, device, context, renderPass, pipelineLayout, subpassId) {
    prepare();
  }

  ~BasicLinePipeline() = default;

  void prepare() override {
    auto& shaders = mPipelineLayout->getShaderModules();

    std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
      {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
      {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
    };

    std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
      {0, sizeof(VertexColor), vk::VertexInputRate::eVertex}
    };

    std::vector<vk::VertexInputAttributeDescription> attributeDescription{
      {0, 0, vk::Format::eR32G32B32Sfloat, 0},
      {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexColor, r)},
    };

    vk::PipelineVertexInputStateCreateInfo inputStateCI({},
                                                        vertBindingDescription,
                                                        attributeDescription);

    vk::PipelineInputAssemblyStateCreateInfo
      inputAssemCI({}, vk::PrimitiveTopology::eLineList, false);

    vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);

    vk::PipelineRasterizationStateCreateInfo rasterizationState;
    rasterizationState.polygonMode = vk::PolygonMode::eFill;
    rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
    rasterizationState.frontFace   = vk::FrontFace::eClockwise;
    rasterizationState.lineWidth   = 4.0f;

    vk::PipelineMultisampleStateCreateInfo multisample_state({},
                                                             vk::SampleCountFlagBits::e1);
    vk::PipelineColorBlendAttachmentState  blendAttachmentState;
    blendAttachmentState.blendEnable         = true;
    blendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
    blendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
    blendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
    blendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
    blendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
    blendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
    blendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
                                          | vk::ColorComponentFlagBits::eG
                                          | vk::ColorComponentFlagBits::eB
                                          | vk::ColorComponentFlagBits::eA;

    vk::PipelineDepthStencilStateCreateInfo depthStencilState;
    depthStencilState.depthCompareOp   = vk::CompareOp::eAlways;
    depthStencilState.depthTestEnable  = true;
    depthStencilState.depthWriteEnable = true;
    depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
    depthStencilState.front            = depthStencilState.back;

    vk::PipelineColorBlendStateCreateInfo colorBlendState({},
                                                          {},
                                                          {},
                                                          blendAttachmentState);

    std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
                                                           vk::DynamicState::eScissor};

    vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);

    vk::GraphicsPipelineCreateInfo pipelineCI({},
                                              shaderStageCIs,
                                              &inputStateCI,
                                              &inputAssemCI,
                                              {},
                                              &viewport_state,
                                              &rasterizationState,
                                              &multisample_state,
                                              &depthStencilState,
                                              &colorBlendState,
                                              &dynamicState,
                                              mPipelineLayout->vk(),
                                              mVkRenderPass,
                                              mSubpassId);

    vk::Result result;

    std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
                                                                       pipelineCI);
    VK_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
  }
};
}  //namespace vkl
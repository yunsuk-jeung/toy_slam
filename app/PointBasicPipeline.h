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
class PointBasicPipeline : public Pipeline {
public:
  PointBasicPipeline(const std::string& name,
                     Device*            device,
                     RenderContext*     context,
                     vk::RenderPass     renderPass,
                     PipelineLayout*    pipelineLayout,
                     uint32_t           subpassId = 0)
    : Pipeline(name, device, context, renderPass, pipelineLayout, subpassId) {}

  ~PointBasicPipeline() = default;

  void PointBasicPipeline::prepareImpl() override {
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
 //{1, 0, vk::Format::eR32G32B32Sfloat, sizeof(float) * 3}
    };

    vk::PipelineVertexInputStateCreateInfo inputStateCI({},
                                                        vertBindingDescription,
                                                        attributeDescription);
    vk::PipelineInputAssemblyStateCreateInfo
      inputAssemCI({}, vk::PrimitiveTopology::ePointList, false);

    auto&                               scProps = mRenderContext->getContextProps();
    vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);

    vk::PipelineRasterizationStateCreateInfo rasterizationState;
    rasterizationState.polygonMode = vk::PolygonMode::eFill;
    rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
    rasterizationState.frontFace   = vk::FrontFace::eClockwise;
    rasterizationState.lineWidth   = 1.0f;

    vk::PipelineMultisampleStateCreateInfo multisample_state({},
                                                             vk::SampleCountFlagBits::e1);

    vk::PipelineDepthStencilStateCreateInfo depthStencilState;
    depthStencilState.depthCompareOp   = vk::CompareOp::eLessOrEqual;
    depthStencilState.depthTestEnable  = true;
    depthStencilState.depthWriteEnable = true;
    depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
    depthStencilState.front            = depthStencilState.back;

    vk::PipelineColorBlendAttachmentState blendAttachmentState;
    blendAttachmentState.colorWriteMask = vk::ColorComponentFlagBits::eR
                                          | vk::ColorComponentFlagBits::eG
                                          | vk::ColorComponentFlagBits::eB
                                          | vk::ColorComponentFlagBits::eA;

    vk::PipelineColorBlendStateCreateInfo colorBlendState({},
                                                          false,
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
                                              {},
                                              {},
                                              -1);

    vk::Result result;

    std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
                                                                       pipelineCI);
    VK_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
  }
};
}  //namespace vkl
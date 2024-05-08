#include "GraphicsPipeline.h"
#include "Device.h"
#include "VklError.h"
#include "Logger.h"
#include "PipelineLayout.h"
#include "RenderContext.h"
#include "ResourcePool.h"
#include "ShaderModule.h"
#include "ShaderTypes.h"
#include <imgui.h>

namespace vkl {
GraphicsPipeline::GraphicsPipeline(const std::string& name,
                                   Device*            device,
                                   RenderContext*     context,
                                   vk::RenderPass     renderPass,
                                   PipelineLayout*    pipelineLayout,
                                   uint32_t           subpassId)
  : mName{name}
  , mDevice{device}
  , mRenderContext{context}
  , mVkRenderPass{renderPass}
  , mPipelineLayout{pipelineLayout}
  , mSubpassId{subpassId}
  , mVertBindingDescription{}
  , mAttributeDescription{}
  , mInputAssemStateCISetted{false}
  , mDynamicStateEnables{} {
  mViewportState = {{}, 1, nullptr, 1, nullptr};

  mRasterizationState.polygonMode = vk::PolygonMode::eFill;
  mRasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
  mRasterizationState.frontFace   = vk::FrontFace::eClockwise;
  mRasterizationState.lineWidth   = 1.0f;

  mMultisampleState = {{}, vk::SampleCountFlagBits::e1};

  mDepthStencilState.depthCompareOp   = vk::CompareOp::eLessOrEqual;
  mDepthStencilState.depthTestEnable  = true;
  mDepthStencilState.depthWriteEnable = true;
  mDepthStencilState.back.compareOp   = vk::CompareOp::eAlways;
  mDepthStencilState.front            = mDepthStencilState.back;

  mBlendAttachmentState.blendEnable         = true;
  mBlendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
  mBlendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
  mBlendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
  mBlendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
  mBlendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
  mBlendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
  mBlendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
                                         | vk::ColorComponentFlagBits::eG
                                         | vk::ColorComponentFlagBits::eB
                                         | vk::ColorComponentFlagBits::eA;

  mColorBlendState = {{}, false, {}, mBlendAttachmentState};

  mDynamicStateEnables = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
}

GraphicsPipeline::~GraphicsPipeline() {
  mDevice->vk().destroyPipeline(this->vk());
}

void GraphicsPipeline::setPrimitiveTopology(vk::PrimitiveTopology topology,
                                            bool                  restart) {
  mInputAssemStateCI       = {{}, topology, restart};
  mInputAssemStateCISetted = true;
}

void GraphicsPipeline::setVertexDescription(
  const std::vector<vk::VertexInputBindingDescription>&   d,
  const std::vector<vk::VertexInputAttributeDescription>& i) {
  mVertBindingDescription = d;
  mAttributeDescription   = i;
}

void GraphicsPipeline::prepare() {
  auto& shaders = mPipelineLayout->getShaderModules();
  std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
    {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
    {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
  };

  VKL_ASSERT_MESSAGE(!mVertBindingDescription.empty(), this->mName.c_str());
  VKL_ASSERT_MESSAGE(!mAttributeDescription.empty(), this->mName.c_str());
  VKL_ASSERT_MESSAGE(mInputAssemStateCISetted, this->mName.c_str());

  vk::PipelineVertexInputStateCreateInfo inputStateCI({},
                                                      mVertBindingDescription,
                                                      mAttributeDescription);

  vk::PipelineDynamicStateCreateInfo dynamicState({}, mDynamicStateEnables);

  vk::GraphicsPipelineCreateInfo pipelineCI({},
                                            shaderStageCIs,
                                            &inputStateCI,
                                            &mInputAssemStateCI,
                                            {},
                                            &mViewportState,
                                            &mRasterizationState,
                                            &mMultisampleState,
                                            &mDepthStencilState,
                                            &mColorBlendState,
                                            &dynamicState,
                                            mPipelineLayout->vk(),
                                            mVkRenderPass,
                                            {},
                                            {},
                                            -1);

  vk::Result result;

  std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
                                                                     pipelineCI);
  VKL_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
}

//
//TextureMaterialPipeline::TextureMaterialPipeline(const std::string& name,
//                                                 Device*            device,
//                                                 RenderContext*     context,
//                                                 vk::RenderPass     renderPass,
//                                                 PipelineLayout*    pipelineLayout,
//                                                 uint32_t           subpassId)
//  : GraphicsPipeline(name, device, context, renderPass, pipelineLayout, subpassId) {
//  prepare();
//}
//
//TextureMaterialPipeline::~TextureMaterialPipeline() {}
//
//void TextureMaterialPipeline::prepare() {
//  auto& shaders = mPipelineLayout->getShaderModules();
//
//  std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
//    {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
//    {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
//  };
//
//  std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
//    {0, sizeof(VertexColorNuv), vk::VertexInputRate::eVertex}
//  };
//
//  std::vector<vk::VertexInputAttributeDescription> attributeDescription{
//    {0, 0, vk::Format::eR32G32B32Sfloat,                 0},
//    {1, 0, vk::Format::eR32G32B32Sfloat, sizeof(float) * 3},
//    {2, 0,    vk::Format::eR32G32Sfloat, sizeof(float) * 6}
//  };
//
//  vk::PipelineVertexInputStateCreateInfo inputStateCI({},
//                                                      vertBindingDescription,
//                                                      attributeDescription);
//  vk::PipelineInputAssemblyStateCreateInfo
//    inputAssemCI({}, vk::PrimitiveTopology::eTriangleList, false);
//
//  auto&                               scProps = mRenderContext->getContextProps();
//  vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);
//
//  vk::PipelineRasterizationStateCreateInfo rasterizationState;
//  rasterizationState.polygonMode = vk::PolygonMode::eFill;
//  rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
//  rasterizationState.frontFace   = vk::FrontFace::eClockwise;
//  rasterizationState.lineWidth   = 1.0f;
//
//  vk::PipelineMultisampleStateCreateInfo multisample_state({},
//                                                           vk::SampleCountFlagBits::e1);
//
//  vk::PipelineDepthStencilStateCreateInfo depthStencilState;
//  depthStencilState.depthCompareOp   = vk::CompareOp::eLessOrEqual;
//  depthStencilState.depthTestEnable  = true;
//  depthStencilState.depthWriteEnable = true;
//  depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
//  depthStencilState.front            = depthStencilState.back;
//
//  vk::PipelineColorBlendAttachmentState blendAttachmentState;
//  blendAttachmentState.colorWriteMask = vk::ColorComponentFlagBits::eR
//                                        | vk::ColorComponentFlagBits::eG
//                                        | vk::ColorComponentFlagBits::eB
//                                        | vk::ColorComponentFlagBits::eA;
//
//  vk::PipelineColorBlendStateCreateInfo colorBlendState({},
//                                                        false,
//                                                        {},
//                                                        blendAttachmentState);
//
//  std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
//                                                         vk::DynamicState::eScissor};
//
//  vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);
//
//  vk::GraphicsPipelineCreateInfo pipelineCI({},
//                                            shaderStageCIs,
//                                            &inputStateCI,
//                                            &inputAssemCI,
//                                            {},
//                                            &viewport_state,
//                                            &rasterizationState,
//                                            &multisample_state,
//                                            &depthStencilState,
//                                            &colorBlendState,
//                                            &dynamicState,
//                                            mPipelineLayout->vk(),
//                                            mVkRenderPass,
//                                            {},
//                                            {},
//                                            -1);
//
//  vk::Result result;
//
//  std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
//                                                                     pipelineCI);
//  VKL_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
//}
//
//ImGuiPipeline::ImGuiPipeline(const std::string& name,
//                             Device*            device,
//                             RenderContext*     context,
//                             vk::RenderPass     renderPass,
//                             PipelineLayout*    pipelineLayout,
//                             uint32_t           subpassId)
//  : Pipeline(name, device, context, renderPass, pipelineLayout, subpassId) {
//  prepare();
//}
//
//ImGuiPipeline::~ImGuiPipeline() {}
//
//void ImGuiPipeline::prepare() {
//  auto* pipelineLayout = ResourcePool::requestPipelineLayout("imgui_vert_imgui_frag");
//  auto& shaders        = pipelineLayout->getShaderModules();
//
//  std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
//    {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
//    {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
//  };
//
//  std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
//    {0, sizeof(ImDrawVert), vk::VertexInputRate::eVertex}
//  };
//
//  std::vector<vk::VertexInputAttributeDescription> attributeDescription{
//    {0, 0,  vk::Format::eR32G32Sfloat, IM_OFFSETOF(ImDrawVert, pos)},
//    {1, 0,  vk::Format::eR32G32Sfloat, IM_OFFSETOF(ImDrawVert,  uv)},
//    {2, 0, vk::Format::eR8G8B8A8Unorm, IM_OFFSETOF(ImDrawVert, col)}
//  };
//
//  vk::PipelineVertexInputStateCreateInfo inputStateCI({},
//                                                      vertBindingDescription,
//                                                      attributeDescription);
//
//  vk::PipelineInputAssemblyStateCreateInfo
//    inputAssemCI({}, vk::PrimitiveTopology::eTriangleList, false);
//
//  vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);
//
//  vk::PipelineRasterizationStateCreateInfo rasterizationState;
//  rasterizationState.polygonMode = vk::PolygonMode::eFill;
//  rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
//  rasterizationState.frontFace   = vk::FrontFace::eClockwise;
//  rasterizationState.lineWidth   = 1.0f;
//
//  vk::PipelineMultisampleStateCreateInfo multisample_state({},
//                                                           vk::SampleCountFlagBits::e1);
//  vk::PipelineColorBlendAttachmentState  blendAttachmentState;
//  blendAttachmentState.blendEnable         = true;
//  blendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
//  blendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
//  blendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
//  blendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
//  blendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
//  blendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
//  blendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
//                                        | vk::ColorComponentFlagBits::eG
//                                        | vk::ColorComponentFlagBits::eB
//                                        | vk::ColorComponentFlagBits::eA;
//
//  vk::PipelineDepthStencilStateCreateInfo depthStencilState;
//  depthStencilState.depthCompareOp   = vk::CompareOp::eAlways;
//  depthStencilState.depthTestEnable  = true;
//  depthStencilState.depthWriteEnable = true;
//  depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
//  depthStencilState.front            = depthStencilState.back;
//
//  vk::PipelineColorBlendStateCreateInfo colorBlendState({}, {}, {},
//  blendAttachmentState);
//
//  std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
//                                                         vk::DynamicState::eScissor};
//
//  vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);
//
//  vk::GraphicsPipelineCreateInfo pipelineCI({},
//                                            shaderStageCIs,
//                                            &inputStateCI,
//                                            &inputAssemCI,
//                                            {},
//                                            &viewport_state,
//                                            &rasterizationState,
//                                            &multisample_state,
//                                            &depthStencilState,
//                                            &colorBlendState,
//                                            &dynamicState,
//                                            pipelineLayout->vk(),
//                                            mVkRenderPass,
//                                            mSubpassId);
//
//  vk::Result result;
//
//  std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
//                                                                     pipelineCI);
//  VKL_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
//}
//
//TrianglePipeline::TrianglePipeline(const std::string& name,
//                                   Device*            device,
//                                   RenderContext*     context,
//                                   vk::RenderPass     renderPass,
//                                   PipelineLayout*    pipelineLayout,
//                                   uint32_t           subpassId)
//  : Pipeline(name, device, context, renderPass, pipelineLayout, subpassId) {
//  prepare();
//}
//
//TrianglePipeline::~TrianglePipeline() {}
//
//void TrianglePipeline::prepare() {
//  auto& shaders = mPipelineLayout->getShaderModules();
//
//  std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
//    {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
//    {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
//  };
//
//  std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
//    {0, sizeof(VertexColor), vk::VertexInputRate::eVertex}
//  };
//
//  std::vector<vk::VertexInputAttributeDescription> attributeDescription{
//    {0, 0, vk::Format::eR32G32B32Sfloat, 0},
//    {1, 0, vk::Format::eR32G32B32Sfloat, offsetof(VertexColor, r)},
//  };
//
//  vk::PipelineVertexInputStateCreateInfo inputStateCI({},
//                                                      vertBindingDescription,
//                                                      attributeDescription);
//
//  vk::PipelineInputAssemblyStateCreateInfo
//    inputAssemCI({}, vk::PrimitiveTopology::eTriangleList, false);
//
//  vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);
//
//  vk::PipelineRasterizationStateCreateInfo rasterizationState;
//  rasterizationState.polygonMode = vk::PolygonMode::eFill;
//  rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
//  rasterizationState.frontFace   = vk::FrontFace::eClockwise;
//  rasterizationState.lineWidth   = 1.0f;
//
//  vk::PipelineMultisampleStateCreateInfo  multisample_state({},
//                                                           vk::SampleCountFlagBits::e1);
//  vk::PipelineDepthStencilStateCreateInfo depthStencilState;
//  depthStencilState.depthCompareOp   = vk::CompareOp::eLessOrEqual;
//  depthStencilState.depthTestEnable  = true;
//  depthStencilState.depthWriteEnable = true;
//  depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
//  depthStencilState.front            = depthStencilState.back;
//
//  vk::PipelineColorBlendAttachmentState blendAttachmentState;
//  blendAttachmentState.blendEnable         = true;
//  blendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
//  blendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
//  blendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
//  blendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
//  blendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
//  blendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
//  blendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
//                                        | vk::ColorComponentFlagBits::eG
//                                        | vk::ColorComponentFlagBits::eB
//                                        | vk::ColorComponentFlagBits::eA;
//
//  vk::PipelineColorBlendStateCreateInfo colorBlendState({}, {}, {},
//  blendAttachmentState);
//
//  std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
//                                                         vk::DynamicState::eScissor};
//
//  vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);
//
//  vk::GraphicsPipelineCreateInfo pipelineCI({},
//                                            shaderStageCIs,
//                                            &inputStateCI,
//                                            &inputAssemCI,
//                                            {},
//                                            &viewport_state,
//                                            &rasterizationState,
//                                            &multisample_state,
//                                            &depthStencilState,
//                                            &colorBlendState,
//                                            &dynamicState,
//                                            mPipelineLayout->vk(),
//                                            mVkRenderPass,
//                                            mSubpassId);
//
//  vk::Result result;
//
//  std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
//                                                                     pipelineCI);
//  VKL_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
//}
//
//TriangleTexPipeline::TriangleTexPipeline(const std::string& name,
//                                         Device*            device,
//                                         RenderContext*     context,
//                                         vk::RenderPass     renderPass,
//                                         PipelineLayout*    pipelineLayout,
//                                         uint32_t           subpassId)
//  : Pipeline(name, device, context, renderPass, pipelineLayout, subpassId) {
//  prepare();
//}
//
//TriangleTexPipeline::~TriangleTexPipeline() {}
//
//void TriangleTexPipeline::prepare() {
//  auto& shaders = mPipelineLayout->getShaderModules();
//
//  std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
//    {{},   vk::ShaderStageFlagBits::eVertex, shaders[0]->vk(), "main"},
//    {{}, vk::ShaderStageFlagBits::eFragment, shaders[1]->vk(), "main"}
//  };
//
//  std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
//    {0, sizeof(VertexNuv), vk::VertexInputRate::eVertex}
//  };
//
//  std::vector<vk::VertexInputAttributeDescription> attributeDescription{
//    {0, 0, vk::Format::eR32G32B32Sfloat, 0},
//    {1, 0, vk::Format::eR32G32Sfloat, offsetof(VertexNuv, nu)},
//  };
//
//  vk::PipelineVertexInputStateCreateInfo inputStateCI({},
//                                                      vertBindingDescription,
//                                                      attributeDescription);
//
//  vk::PipelineInputAssemblyStateCreateInfo
//    inputAssemCI({}, vk::PrimitiveTopology::eTriangleList, false);
//
//  vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);
//
//  vk::PipelineRasterizationStateCreateInfo rasterizationState;
//  rasterizationState.polygonMode = vk::PolygonMode::eFill;
//  rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
//  rasterizationState.frontFace   = vk::FrontFace::eClockwise;
//  rasterizationState.lineWidth   = 1.0f;
//
//  vk::PipelineMultisampleStateCreateInfo multisample_state({},
//                                                           vk::SampleCountFlagBits::e1);
//  vk::PipelineColorBlendAttachmentState  blendAttachmentState;
//  blendAttachmentState.blendEnable         = true;
//  blendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
//  blendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
//  blendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
//  blendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
//  blendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
//  blendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
//  blendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
//                                        | vk::ColorComponentFlagBits::eG
//                                        | vk::ColorComponentFlagBits::eB
//                                        | vk::ColorComponentFlagBits::eA;
//
//  vk::PipelineDepthStencilStateCreateInfo depthStencilState;
//  depthStencilState.depthCompareOp   = vk::CompareOp::eLessOrEqual;
//  depthStencilState.depthTestEnable  = true;
//  depthStencilState.depthWriteEnable = true;
//  depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
//  depthStencilState.front            = depthStencilState.back;
//
//  vk::PipelineColorBlendStateCreateInfo colorBlendState({}, {}, {},
//  blendAttachmentState);
//
//  std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
//                                                         vk::DynamicState::eScissor};
//
//  vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);
//
//  vk::GraphicsPipelineCreateInfo pipelineCI({},
//                                            shaderStageCIs,
//                                            &inputStateCI,
//                                            &inputAssemCI,
//                                            {},
//                                            &viewport_state,
//                                            &rasterizationState,
//                                            &multisample_state,
//                                            &depthStencilState,
//                                            &colorBlendState,
//                                            &dynamicState,
//                                            mPipelineLayout->vk(),
//                                            mVkRenderPass,
//                                            mSubpassId);
//
//  vk::Result result;
//
//  std::tie(result, mVkObject) = mDevice->vk().createGraphicsPipeline(VK_NULL_HANDLE,
//                                                                     pipelineCI);
//  VKL_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
//}

}  //namespace vkl

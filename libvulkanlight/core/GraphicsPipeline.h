#pragma once
#include "VkObject.h"
#include "macros.h"
#include <string>

namespace vkl {
class Device;
class RenderContext;
class PipelineLayout;
class GraphicsPipeline : public VkObject<vk::Pipeline> {
public:
  USING_SMART_PTR(GraphicsPipeline);
  GraphicsPipeline() = delete;
  GraphicsPipeline(const std::string& name,
                   Device*            device,
                   RenderContext*     context,
                   vk::RenderPass     renderPass,
                   PipelineLayout*    pipelineLayout,
                   uint32_t           subpassId);
  virtual ~GraphicsPipeline();

  void setPrimitiveTopology(vk::PrimitiveTopology topology, bool restart = false);
  void setVertexDescription(const std::vector<vk::VertexInputBindingDescription>&   d,
                            const std::vector<vk::VertexInputAttributeDescription>& i);

  void prepare();

protected:
  //void addResource();

protected:
  std::string     mName;
  Device*         mDevice;
  RenderContext*  mRenderContext;
  vk::RenderPass  mVkRenderPass;
  PipelineLayout* mPipelineLayout;
  uint32_t        mSubpassId;

  std::vector<vk::VertexInputBindingDescription>   mVertBindingDescription;
  std::vector<vk::VertexInputAttributeDescription> mAttributeDescription;
  vk::PipelineInputAssemblyStateCreateInfo         mInputAssemStateCI;
  bool                                             mInputAssemStateCISetted;
  vk::PipelineViewportStateCreateInfo              mViewportState;
  vk::PipelineRasterizationStateCreateInfo         mRasterizationState;
  vk::PipelineMultisampleStateCreateInfo           mMultisampleState;
  vk::PipelineDepthStencilStateCreateInfo          mDepthStencilState;
  vk::PipelineColorBlendAttachmentState            mBlendAttachmentState;
  vk::PipelineColorBlendStateCreateInfo            mColorBlendState;
  std::vector<vk::DynamicState>                    mDynamicStateEnables;

public:
  PipelineLayout* getPipelineLayout() { return mPipelineLayout; }
  auto&           getRasterizationStateCI() { return mRasterizationState; }
};

//class TextureMaterialPipeline : public GraphicsPipeline {
//public:
//  TextureMaterialPipeline() = delete;
//  TextureMaterialPipeline(const std::string& name,
//                          Device*            device,
//                          RenderContext*     context,
//                          vk::RenderPass     renderPass,
//                          PipelineLayout*    pipelineLayout,
//                          uint32_t           subpassId = 0);
//  ~TextureMaterialPipeline();
//
//protected:
//  void prepare() override;
//};
//
//class ImGuiPipeline : public GraphicsPipeline {
//public:
//  ImGuiPipeline() = delete;
//  ImGuiPipeline(const std::string& name,
//                Device*            device,
//                RenderContext*     context,
//                vk::RenderPass     renderPass,
//                PipelineLayout*    pipelineLayout,
//                uint32_t           subpassId = 0);
//  ~ImGuiPipeline();
//
//protected:
//  void prepare() override;
//};
//
//class TrianglePipeline : public GraphicsPipeline {
//public:
//  TrianglePipeline() = delete;
//  TrianglePipeline(const std::string& name,
//                   Device*            device,
//                   RenderContext*     context,
//                   vk::RenderPass     renderPass,
//                   PipelineLayout*    pipelineLayout,
//                   uint32_t           subpassId = 0);
//  ~TrianglePipeline();
//
//protected:
//  void prepare() override;
//};
//
//class TriangleTexPipeline : public GraphicsPipeline {
//public:
//  TriangleTexPipeline() = delete;
//  TriangleTexPipeline(const std::string& name,
//                      Device*            device,
//                      RenderContext*     context,
//                      vk::RenderPass     renderPass,
//                      PipelineLayout*    pipelineLayout,
//                      uint32_t           subpassId = 0);
//  ~TriangleTexPipeline();
//
//protected:
//  void prepare() override;
//};

}  //namespace vkl
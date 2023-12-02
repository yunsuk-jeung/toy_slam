#pragma once
#include <string>
#include "macros.h"
#include "VkObject.h"
namespace vkl {
class Device;
class RenderContext;
class PipelineLayout;
class Pipeline : public VkObject<vk::Pipeline> {
public:
  USING_SMART_PTR(Pipeline);
  Pipeline() = delete;
  Pipeline(Device*         device,
           RenderContext*  context,
           vk::RenderPass  renderPass,
           PipelineLayout* pipelineLayout,
           uint32_t        subpassId);
  virtual ~Pipeline();
  virtual void prepare() = 0;

protected:
  void addResource();

protected:
  std::string     mName;
  Device*         mDevice;
  RenderContext*  mRenderContext;
  vk::RenderPass  mVkRenderPass;
  PipelineLayout* mPipelineLayout;
  uint32_t        mSubpassId;

public:
  PipelineLayout* getPipelineLayout() { return mPipelineLayout; }
};

class BasicTraianglePipeline : public Pipeline {
public:
  BasicTraianglePipeline() = delete;
  BasicTraianglePipeline(const std::string& name,
                         Device*            device,
                         RenderContext*     context,
                         vk::RenderPass     renderPass,
                         PipelineLayout*    pipelineLayout,
                         uint32_t           subpassId = 0);
  ~BasicTraianglePipeline();

  void prepare() override;
};

class ImGuiPipeline : public Pipeline {
public:
  ImGuiPipeline() = delete;
  ImGuiPipeline(const std::string& name,
                Device*            device,
                RenderContext*     context,
                vk::RenderPass     renderPass,
                PipelineLayout*    pipelineLayout,
                uint32_t           subpassId = 0);
  ~ImGuiPipeline();

  void prepare() override;
};

}  //namespace vkl
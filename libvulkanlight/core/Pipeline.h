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
           PipelineLayout* pipelineLayout);
  virtual ~Pipeline();
  virtual void prepare() = 0;

protected:
  std::string     mName;
  Device*         mDevice;
  RenderContext*  mRenderContext;
  vk::RenderPass  mVkRenderPass;
  PipelineLayout* mPipelineLayout;
};

class BasicTraianglePipeline : public Pipeline {
public:
  BasicTraianglePipeline() = delete;
  BasicTraianglePipeline(const std::string& name,
                         Device*            device,
                         RenderContext*     context,
                         vk::RenderPass     renderPass,
                         PipelineLayout*    pipelineLayout);
  ~BasicTraianglePipeline();

  void prepare() override;
};

}  //namespace vkl
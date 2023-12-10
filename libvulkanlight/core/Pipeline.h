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
  Pipeline(const std::string& name,
           Device*            device,
           RenderContext*     context,
           vk::RenderPass     renderPass,
           PipelineLayout*    pipelineLayout,
           uint32_t           subpassId);
  virtual ~Pipeline();

protected:
  virtual void prepare() = 0;
  //void addResource();

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

class TextureMaterialPipeline : public Pipeline {
public:
  TextureMaterialPipeline() = delete;
  TextureMaterialPipeline(const std::string& name,
                          Device*            device,
                          RenderContext*     context,
                          vk::RenderPass     renderPass,
                          PipelineLayout*    pipelineLayout,
                          uint32_t           subpassId = 0);
  ~TextureMaterialPipeline();

protected:
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

protected:
  void prepare() override;
};

class SamplePipeline : public Pipeline {
public:
  SamplePipeline() = delete;
  SamplePipeline(const std::string& name,
                 Device*            device,
                 RenderContext*     context,
                 vk::RenderPass     renderPass,
                 PipelineLayout*    pipelineLayout,
                 uint32_t           subpassId = 0);
  ~SamplePipeline();

protected:
  void prepare() override;
};
}  //namespace vkl
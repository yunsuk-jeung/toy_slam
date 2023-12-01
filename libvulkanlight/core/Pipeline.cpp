#include "Device.h"
#include "PipelineLayout.h"
#include "Pipeline.h"

namespace vkl {
Pipeline::Pipeline(Device*         device,
                   vk::RenderPass  renderPass,
                   PipelineLayout* pipelineLayout)
  : mDevice{device}
  , mRenderPass{renderPass}
  , mPipelineLayout{pipelineLayout} {}

Pipeline::~Pipeline() {
  mDevice->vk().destroyPipeline(mVkObject);
}
}  //namespace vkl

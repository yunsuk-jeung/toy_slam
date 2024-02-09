#include "Device.h"
#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "ComputePipeline.h"
namespace vkl {
ComputePipeline::ComputePipeline(const std::string& name,
                                 Device*            device,
                                 PipelineLayout*    pipelineLayout)
  : mName{name}
  , mDevice{device}
  , mPipelineLayout{pipelineLayout} {}

void ComputePipeline::prepare() {
  auto&                         shaders = mPipelineLayout->getShaderModules();
  vk::ComputePipelineCreateInfo ci;
  ci.stage  = {{}, vk::ShaderStageFlagBits::eCompute, shaders[0]->vk(), "main"};
  ci.layout = mPipelineLayout->vk();
  vk::Result result;
  std::tie(result, mVkObject) = mDevice->vk().createComputePipeline({}, ci);
}

}  //namespace vkl
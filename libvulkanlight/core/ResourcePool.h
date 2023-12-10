#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include "VklLogger.h"
#include "vkltypes.h"
#include "Pipeline.h"

namespace vkl {
class Device;
class RenderContext;
class PipelineLayout;
class Pipeline;
class ShaderModule;
class ResourcePool {
public:
  static void clear(Device* device);

  static void                      addShaderPath(std::string path);
  static std::vector<std::string>& getShaderPaths();

  static void                      addResourcePath(std::string path);
  static std::vector<std::string>& getResourcePaths();

  static ShaderModule* loadShader(const std::string&      name,
                                  Device*                 device,
                                  ShaderSourceType        srcType,
                                  vk::ShaderStageFlagBits stage,
                                  const std::string&      shaderSrc);

  static void                    addDescriptorSetLayout(const std::string&      name,
                                                        vk::DescriptorSetLayout layout);
  static vk::DescriptorSetLayout requestDescriptorSetLayout(const std::string& name);

  static PipelineLayout* addPipelineLayout(Device*       device,
                                           ShaderModule* vert,
                                           ShaderModule* frag);
  static PipelineLayout* requestPipelineLayout(const std::string& name);

  template <typename PipelineType>
  static Pipeline* addPipeline(const std::string& name,
                               Device*            device,
                               RenderContext*     context,
                               vk::RenderPass     renderPass,
                               PipelineLayout*    pipelineLayout,
                               uint32_t           subpassId = 0) {
    const std::string& hash = name;

    std::hash<std::string> hasher;
    size_t                 key = hasher(hash);
    auto                   it  = mPipelines.find(key);
    if (it != mPipelines.end()) {
      VklLogE("you are adding existing pipeline: {}", name);
      throw std::runtime_error("existing pipeline");
    }

    mPipelines.insert({key,
                       std::make_unique<PipelineType>(name,
                                                      device,
                                                      context,
                                                      renderPass,
                                                      pipelineLayout,
                                                      subpassId)});
    //mPipelines[key]->prepare();

    return mPipelines[key].get();
  }

  static Pipeline* requestPipeline(const std::string& name);

protected:

protected:
  static std::vector<std::string>                                  mShaderPaths;
  static std::vector<std::string>                                  mResourcePaths;
  static std::unordered_map<size_t, std::unique_ptr<ShaderModule>> mShaderPools;
  static std::unordered_map<size_t, vk::DescriptorSetLayout>       mDescriptorSetLayouts;
  static std::unordered_map<size_t, std::unique_ptr<PipelineLayout>> mPipelineLayouts;
  static std::unordered_map<size_t, std::unique_ptr<Pipeline>>       mPipelines;
};
}  //namespace vkl

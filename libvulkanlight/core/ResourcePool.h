#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include "vkltypes.h"

namespace vkl {
class Device;
class PipelineLayout;
class Pipeline;
class ShaderModule;
class ResourcePool {
public:
  static void clear(Device* device);

  static void         setShaderPath(std::string path);
  static std::string& getShaderPath();

  static void         setResourcePath(std::string path);
  static std::string& getResourcePath();

  static ShaderModule* loadShader(const std::string&      name,
                                  Device*                 device,
                                  ShaderSourceType        srcType,
                                  vk::ShaderStageFlagBits stage,
                                  const std::string&      shaderSrc);

  static void addDescriptorSetLayout(const std::string&      name,
                                     vk::DescriptorSetLayout descSetLayout);

  static vk::DescriptorSetLayout requestDescriptorSetLayout(const std::string& name);

  static void addPipelineLayout(Device* device, ShaderModule* vert, ShaderModule* frag);

  static PipelineLayout* requestPipelineLayout(const std::string& name);

  static void addPipeline(const std::string& name, Pipeline* pipeline);

  static Pipeline* requestPipeline(const std::string& name);

protected:

protected:
  static std::string                                               mShaderPath;
  static std::string                                               mResourcePath;
  static std::unordered_map<size_t, std::unique_ptr<ShaderModule>> mShaderPools;
  static std::unordered_map<size_t, vk::DescriptorSetLayout>       mDescriptorSetLayouts;
  static std::unordered_map<size_t, std::unique_ptr<PipelineLayout>> mPipelineLayouts;
  static std::unordered_map<size_t, std::unique_ptr<Pipeline>>       mPipelines;
};
}  //namespace vkl

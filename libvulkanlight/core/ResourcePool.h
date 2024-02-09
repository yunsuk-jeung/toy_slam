#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vulkan/vulkan.hpp>
#include "VklLogger.h"
#include "vkltypes.h"

namespace vkl {
class Device;
class RenderContext;
class DescriptorSetLayout;
class PipelineLayout;
class GraphicsPipeline;
class ShaderModule;
class ResourcePool {
public:
  static void init(Device* _device) { device = _device; }
  static void clear();

  static void                      addShaderPath(std::string path);
  static std::vector<std::string>& getShaderPaths();

  static void                      addAssetPath(std::string path);
  static std::vector<std::string>& getAssetPaths();

  static ShaderModule* loadShader(const std::string&      name,
                                  ShaderSourceType        srcType,
                                  vk::ShaderStageFlagBits stage);

  static DescriptorSetLayout* addDescriptorSetLayout(
    const std::string&                                 name,
    const std::vector<vk::DescriptorSetLayoutBinding>& bindings);
  static DescriptorSetLayout* requestDescriptorSetLayout(const std::string& name);

  static PipelineLayout* addPipelineLayout(ShaderModule*      comp,
                                           const std::string& tag = "");

  static PipelineLayout* addPipelineLayout(ShaderModule*      vert,
                                           ShaderModule*      frag,
                                           const std::string& tag = "");
  static PipelineLayout* requestPipelineLayout(const std::string& name);

  static vk::Sampler addSampler(const std::string& name, vk::SamplerCreateInfo samplerCI);
  static vk::Sampler requestSampler(const std::string& name);

  static GraphicsPipeline* addGraphicsPipeline(const std::string& name,
                                               RenderContext*     context,
                                               vk::RenderPass     renderPass,
                                               PipelineLayout*    pipelineLayout,
                                               uint32_t           subpassId = 0);

  static GraphicsPipeline* requestGraphicsPipeline(const std::string& name);
  static void              deleteGraphicsPipeline(const std::string& name);

protected:

protected:
  static Device* device;
  // clang-format off
  static std::vector<std::string>                                         mShaderPaths;
  static std::vector<std::string>                                         mAssetPaths;
  static std::unordered_map<size_t, std::unique_ptr<ShaderModule>>        mShaderModules;
  static std::unordered_map<size_t, std::unique_ptr<DescriptorSetLayout>> mDescriptorSetLayouts;
  static std::unordered_map<size_t, vk::Sampler>                          mSamplers;
  static std::unordered_map<size_t, std::unique_ptr<PipelineLayout>>      mPipelineLayouts;
  static std::unordered_map<size_t, std::unique_ptr<GraphicsPipeline>>    mGraphicsPipelines;
  // clang-format on
};
}  //namespace vkl

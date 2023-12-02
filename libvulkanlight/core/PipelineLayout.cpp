#include <map>
#include "VkLogger.h"
#include "ResourcePool.h"
#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "Device.h"
namespace vkl {

namespace {
inline vk::DescriptorType find_descriptor_type(ShaderResourceType resource_type,
                                               bool               dynamic) {
  switch (resource_type) {
  case ShaderResourceType::InputAttachment:
    return vk::DescriptorType::eInputAttachment;
    break;
  case ShaderResourceType::Image:
    return vk::DescriptorType::eSampledImage;
    break;
  case ShaderResourceType::ImageSampler:
    return vk::DescriptorType::eCombinedImageSampler;
    break;
  case ShaderResourceType::ImageStorage:
    return vk::DescriptorType::eStorageImage;
    break;
  case ShaderResourceType::Sampler:
    return vk::DescriptorType::eSampler;
    break;
  case ShaderResourceType::BufferUniform:
    if (dynamic) { return vk::DescriptorType::eUniformBufferDynamic; }
    else { return vk::DescriptorType::eUniformBuffer; }
    break;
  case ShaderResourceType::BufferStorage:
    if (dynamic) { return vk::DescriptorType::eStorageBufferDynamic; }
    else { return vk::DescriptorType::eStorageBuffer; }
    break;
  default:
    throw std::runtime_error("No conversion possible for the shader resource type.");
    break;
  }
}
}  //namespace

std::vector<vk::DescriptorSetLayout> PipelineLayout::prepareDescSetLayouts() {
  std::vector<vk::DescriptorSetLayout>                       layouts;
  std::map<int, std::vector<vk::DescriptorSetLayoutBinding>> bindingsMap;
  std::map<int, std::vector<std::string>>                    namesMap;
  std::map<int, std::string>                                 shaderNameMap;
  for (auto& shader : mShaderModules) {
    auto& shaderResources = shader->getShaderResources();
    for (auto& resource : shaderResources) {
      if (resource.type == ShaderResourceType::Input
          || resource.type == ShaderResourceType::Output
          || resource.type == ShaderResourceType::PushConstant
          || resource.type == ShaderResourceType::SpecializationConstant) {
        continue;
      }

      auto descriptor_type = find_descriptor_type(resource.type,
                                                  resource.mode
                                                    == ShaderResourceMode::Dynamic);

      vk::DescriptorSetLayoutBinding layoutBinding;
      layoutBinding.binding         = resource.binding;
      layoutBinding.descriptorCount = resource.array_size;
      layoutBinding.descriptorType  = descriptor_type;
      layoutBinding.stageFlags      = static_cast<vk::ShaderStageFlags>(resource.stages);

      auto name = "_" + resource.name;
      VklLogD("desc set : {} binding : {} name : {}",
              resource.set,
              resource.binding,
              resource.name);
      bindingsMap[resource.set].push_back(layoutBinding);
      namesMap[resource.set].push_back(name);
      shaderNameMap[resource.set] = shader->getName();
    }
  }
  for (auto& [key, bindings] : bindingsMap) {
    auto descSetLayout = mDevice->vk().createDescriptorSetLayout({{}, bindings});

    std::string name  = shaderNameMap[key];
    auto&       names = namesMap[key];
    for (auto& n : names) { name += n; }

    VklLogD("name : {} desc set : {} binding num : {}", name, key, bindings.size());
    ResourcePool::addDescriptorSetLayout(name, descSetLayout);
    layouts.push_back(descSetLayout);
  }

  return layouts;
}

PipelineLayout::PipelineLayout(Device* device, std::vector<ShaderModule*>& shaderModules)
  : mDevice{device}
  , mName{""} {

  for (auto& shader : shaderModules) { mName += shader->getName() + "_"; }
  mName.pop_back();

  mShaderModules = shaderModules;

  auto layouts = prepareDescSetLayouts();

  std::vector<vk::PushConstantRange> pushConstRanges;
  for (auto& shader : mShaderModules) {
    auto& shaderResources = shader->getShaderResources();
    for (auto& resource : shaderResources) {
      if (resource.type != ShaderResourceType::PushConstant) continue;
      pushConstRanges.push_back({resource.stages, resource.offset, resource.size});
    }
  }

  vk::PipelineLayoutCreateInfo CI({}, layouts, pushConstRanges);

  mVkObject = mDevice->vk().createPipelineLayout(CI);
}

PipelineLayout::~PipelineLayout() {
  mDevice->vk().destroyPipelineLayout(mVkObject);
}
}  //namespace vkl
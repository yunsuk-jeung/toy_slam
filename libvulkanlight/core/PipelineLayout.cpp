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
  std::vector<vk::DescriptorSetLayout> layouts;
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

      auto descSet = mDevice->getVkDevice().createDescriptorSetLayout(
        {{}, layoutBinding});
      auto name = mName + resource.name;
      ResourcePool::addDescriptorSetLayouts(name, descSet);
      layouts.push_back(descSet);
    }
  }
  return layouts;
}

PipelineLayout::PipelineLayout(Device* device, std::vector<ShaderModule*>& shaderModules)
  : mDevice{device}
  , mName{""} {
  for (auto& shader : shaderModules) { mName += shader->getName(); }

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
  mDevice->getVkDevice().createPipelineLayout(CI);
}
}  //namespace vkl
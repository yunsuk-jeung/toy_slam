#pragma once
#include "VkLogger.h"
#include "Device.h"
#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "VkShaderUtil.h"
#include "ResourcePool.h"
#include "Utils.h"
#include "SPIRVReflection.h"

namespace vkl {
std::unordered_map<size_t, ShaderModule::Uni>       ResourcePool::mShaderPools;
std::unordered_map<size_t, vk::DescriptorSetLayout> ResourcePool::mDescriptorSetLayouts;
std::unordered_map<size_t, PipelineLayout::Uni>     ResourcePool::mPipelineLayouts;
std::unordered_map<size_t, Pipeline::Uni>           ResourcePool::mPipelines;

void ResourcePool::clear(Device* device) {
  mShaderPools.clear();
  for (auto& [key, layout] : mDescriptorSetLayouts) {
    device->vk().destroyDescriptorSetLayout(layout);
  }
  mPipelineLayouts.clear();

  for (auto& [key, pipeline] : mPipelines) {
    device->vk().destroyPipeline(pipeline->vk());
  }
}

ShaderModule* ResourcePool::loadShader(const std::string&      name,
                                       Device*                 device,
                                       ShaderSourceType        srcType,
                                       vk::ShaderStageFlagBits stage,
                                       const std::string&      shaderSrc) {
  std::string type = stage == vk::ShaderStageFlagBits::eVertex ? "_vert" : "_frag";
  std::string hash = name + type;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);

  bool createNew = false;
  if (mShaderPools.find(key) == mShaderPools.end()) {
    mShaderPools.insert({key, ShaderModule::Uni(new ShaderModule(device, stage))});
    createNew = true;
  }

  ShaderModule* out = mShaderPools[key].get();
  if (!createNew) return out;

  out->setName(hash);

  std::vector<uint32_t> spirv;
  switch (srcType) {
  case ShaderSourceType::STRING_FILE: {
    const std::string& shaderFolderPath = Utils::getShaderPath();
    std::string        shaderFile       = shaderFolderPath + "/" + shaderSrc;
    out->vk() = VkShaderUtil::loadShader(device->vk(), shaderFile, spirv);
    break;
  }
  case ShaderSourceType::STRING: {
    out->vk() = VkShaderUtil::loadShader(device->vk(), shaderSrc, stage, spirv);
    break;
  }
  case ShaderSourceType::SPV_FILE:
    break;
  case ShaderSourceType::SPV: {
    uint32_t* src  = (uint32_t*)shaderSrc.c_str();
    size_t    size = shaderSrc.length();
    out->vk()      = VkShaderUtil::loadShader(device->vk(), src, size, spirv);
    break;
  }
  default:
    break;
  }

  out->setSpirv(spirv);
  SPIRVReflection::reflectShaderRresources(out->getStage(),
                                           out->getSpirv(),
                                           out->getShaderResources(),
                                           {});
  return out;
}

void ResourcePool::addDescriptorSetLayouts(const std::string&      name,
                                           vk::DescriptorSetLayout descSetLayout) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);
  mDescriptorSetLayouts.insert({key, descSetLayout});
}

void ResourcePool::addPipelineLayout(Device*       device,
                                     ShaderModule* vert,
                                     ShaderModule* frag) {
  std::vector<ShaderModule*> shaders{vert, frag};

  std::string            name = vert->getName() + "_" + frag->getName();
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);
  auto                   it  = mPipelineLayouts.find(key);

  if (mPipelineLayouts.find(key) != mPipelineLayouts.end()) {
    VklLogE("you are adding existing pipeline: {}", name);
    throw std::runtime_error("existing pipeline");
  }
  mPipelineLayouts.insert(
    {key, PipelineLayout::Uni(new PipelineLayout(device, shaders))});
}

PipelineLayout* ResourcePool::requestPipelineLayout(const std::string& name) {
  const std::string& hash = name;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mPipelineLayouts.find(key);

  if (it == mPipelineLayouts.end()) {
    VklLogE("you are requesting non existing pipelineLayout : {}", name);
    throw std::runtime_error("non existing pipelineLayout");
  }

  return it->second.get();
}

void ResourcePool::addPipeline(const std::string& name, Pipeline* pipeline) {
  const std::string& hash = name;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mPipelines.find(key);
  if (it != mPipelines.end()) {
    VklLogE("you are adding existing pipeline: {}", name);
    throw std::runtime_error("existing pipeline");
  }

  mPipelines[key] = Pipeline::Uni(pipeline);
}

Pipeline* ResourcePool::requestPipeline(const std::string& name) {
  const std::string& hash = name;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mPipelines.find(key);
  if (it == mPipelines.end()) {
    VklLogE("you are requesting none existing pipeline: {}", name);
    throw std::runtime_error("none existing pipeline");
  }

  return mPipelines[key].get();
}

}  //namespace vkl
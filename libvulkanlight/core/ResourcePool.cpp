#pragma once

#include "ResourcePool.h"
#include "Device.h"
#include "Logger.h"
#include "VklError.h"
#include "DescriptorSetLayout.h"
#include "GraphicsPipeline.h"
#include "PipelineLayout.h"
#include "SPIRVReflection.h"
#include "ShaderModule.h"
#include "VklShaderUtil.h"
#include "ShaderPool.h"
#include "common.h"

namespace vkl {
Device* ResourcePool::device{nullptr};
// clang-format off
std::vector<std::string>                                         ResourcePool::mShaderPaths;
std::vector<std::string>                                         ResourcePool::mAssetPaths;
std::unordered_map<size_t, ShaderModule::Uni>                    ResourcePool::mShaderModules;
std::unordered_map<size_t, std::unique_ptr<DescriptorSetLayout>> ResourcePool::mDescriptorSetLayouts;
std::unordered_map<size_t, vk::Sampler>                          ResourcePool::mSamplers;
std::unordered_map<size_t, PipelineLayout::Uni>                  ResourcePool::mPipelineLayouts;
std::unordered_map<size_t, GraphicsPipeline::Uni>                ResourcePool::mGraphicsPipelines;
// clang-format on

void ResourcePool::clear() {
  mShaderModules.clear();
  mDescriptorSetLayouts.clear();

  for (auto& [key, sampler] : mSamplers) {
    device->vk().destroySampler(sampler);
  }

  mPipelineLayouts.clear();
  mGraphicsPipelines.clear();
}

void ResourcePool::addShaderPath(std::string path) {
  mShaderPaths.push_back(path);
}

std::vector<std::string>& ResourcePool::getShaderPaths() {
  return mShaderPaths;
}

void ResourcePool::addAssetPath(std::string path) {
  mAssetPaths.push_back(path);
}

std::vector<std::string>& ResourcePool::getAssetPaths() {
  return mAssetPaths;
}

ShaderModule* ResourcePool::loadShader(const std::string&      name,
                                       ShaderSourceType        srcType,
                                       vk::ShaderStageFlagBits stage) {
  std::string type;
  switch (stage) {
  case vk::ShaderStageFlagBits::eVertex:
    type = "_vert";
    break;
  case vk::ShaderStageFlagBits::eFragment:
    type = "_frag";
    break;
  case vk::ShaderStageFlagBits::eCompute:
    type = "_comp";
    break;
  }
  std::string hash = name + type;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);

  bool createNew = false;
  if (mShaderModules.find(key) == mShaderModules.end()) {
    mShaderModules.insert({key, std::make_unique<ShaderModule>(device, stage)});
    createNew = true;
  }

  ShaderModule* out = mShaderModules[key].get();
  if (!createNew)
    return out;

  out->setName(hash);

  std::vector<uint32_t> spirv;
  switch (srcType) {
  case ShaderSourceType::STRING_FILE: {
    std::string ext;
    switch (stage) {
    case vk::ShaderStageFlagBits::eVertex:
      ext = ".vert";
      break;
    case vk::ShaderStageFlagBits::eFragment:
      ext = ".frag";
      break;
    case vk::ShaderStageFlagBits::eCompute:
      ext = ".comp";
      break;
    }
    std::string file = name + ext;

    for (const std::string& shaderFolderPath : ResourcePool::getShaderPaths()) {
      std::string shaderFile = shaderFolderPath + "/" + file;
      if (!fs::fileExists(shaderFile))
        continue;
      out->vk() = VklShaderUtil::loadShader(device->vk(), shaderFile, spirv);
    }
    break;

    break;
  }
  case ShaderSourceType::STRING: {
    auto str  = ShaderPool::requestShaderStr(hash);
    out->vk() = VklShaderUtil::loadShader(device->vk(), str, stage, spirv);
    break;
  }
  case ShaderSourceType::SPV_FILE:
    break;
  case ShaderSourceType::SPV: {
    auto      str  = ShaderPool::requestShaderStr(hash);
    uint32_t* src  = (uint32_t*)str.c_str();
    size_t    size = str.length();
    out->vk()      = VklShaderUtil::loadShader(device->vk(), src, size, spirv);
    break;
  }
  default:
    break;
  }

  if (!out->vk()) {
    VKL_ASSERT_MESSAGE(0, "shader not loaded");
  }
  out->setSpirv(spirv);
  SPIRVReflection::reflectShaderRresources(out->getStage(),
                                           out->getSpirv(),
                                           out->getShaderResources(),
                                           {});
  return out;
}

DescriptorSetLayout* ResourcePool::addDescriptorSetLayout(
  const std::string&                                 name,
  const std::vector<vk::DescriptorSetLayoutBinding>& bindings) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);

  auto it = mDescriptorSetLayouts.find(key);
  if (it != mDescriptorSetLayouts.end()) {
    vklLogW("you are adding existing descsetlayout: {}", name);
    return mDescriptorSetLayouts[key].get();
  }
  DescriptorSetLayout a(device, bindings);

  mDescriptorSetLayouts.insert(
    {key, std::make_unique<DescriptorSetLayout>(device, bindings)});

  return mDescriptorSetLayouts[key].get();
}

DescriptorSetLayout* ResourcePool::requestDescriptorSetLayout(const std::string& name) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);

  auto it = mDescriptorSetLayouts.find(key);
  if (it == mDescriptorSetLayouts.end()) {
    vklLogE("you are requesting none existing descsetlayout: {}", name);
    throw std::runtime_error("none existing descsetlayout");
  }

  return it->second.get();
}

vk::Sampler vkl::ResourcePool::addSampler(const std::string&    name,
                                          vk::SamplerCreateInfo samplerCI) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);
  auto                   it  = mSamplers.find(key);
  if (it != mSamplers.end()) {
    vklLogW("you are adding existing sampler: {}", name);
    return mSamplers[key];
  }

  auto sampler = device->vk().createSampler(samplerCI);
  mSamplers.insert({key, sampler});
  return mSamplers[key];
}

vk::Sampler ResourcePool::requestSampler(const std::string& name) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);

  auto it = mSamplers.find(key);
  if (it == mSamplers.end()) {
    vklLogE("you are requesting none existing sampler: {}", name);
    throw std::runtime_error("none existing sampler");
  }

  return it->second;
}

PipelineLayout* ResourcePool::addPipelineLayout(ShaderModule*      comp,
                                                const std::string& tag) {
  std::vector<ShaderModule*> shaders{comp};

  std::string name = comp->getName();

  if (tag.length() > 0) {
    name = tag + "_" + name;
  }

  std::hash<std::string> hasher;
  size_t                 key = hasher(name);
  auto                   it  = mPipelineLayouts.find(key);

  if (mPipelineLayouts.find(key) != mPipelineLayouts.end()) {
    vklLogE("you are adding existing pipelinelayout: {}", name);
    return mPipelineLayouts[key].get();
  }
  mPipelineLayouts.insert({key, std::make_unique<PipelineLayout>(device, shaders)});
  return mPipelineLayouts[key].get();
}

PipelineLayout* ResourcePool::addPipelineLayout(ShaderModule*      vert,
                                                ShaderModule*      frag,
                                                const std::string& tag) {
  std::vector<ShaderModule*> shaders{vert, frag};

  std::string name = vert->getName() + "_" + frag->getName();

  if (tag.length() > 0) {
    name = tag + "_" + name;
  }

  std::hash<std::string> hasher;
  size_t                 key = hasher(name);
  auto                   it  = mPipelineLayouts.find(key);

  if (mPipelineLayouts.find(key) != mPipelineLayouts.end()) {
    vklLogE("you are adding existing pipelinelayout: {}", name);
    return mPipelineLayouts[key].get();
  }
  mPipelineLayouts.insert({key, std::make_unique<PipelineLayout>(device, shaders)});
  return mPipelineLayouts[key].get();
}

PipelineLayout* ResourcePool::requestPipelineLayout(const std::string& name) {
  const std::string& hash = name;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mPipelineLayouts.find(key);

  if (it == mPipelineLayouts.end()) {
    vklLogE("you are requesting non existing pipelineLayout : {}", name);
    throw std::runtime_error("non existing pipelineLayout");
  }

  return it->second.get();
}

GraphicsPipeline* ResourcePool::addGraphicsPipeline(const std::string& name,
                                                    RenderContext*     context,
                                                    vk::RenderPass     renderPass,
                                                    PipelineLayout*    pipelineLayout,
                                                    uint32_t           subpassId) {
  const std::string& hash = name;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mGraphicsPipelines.find(key);
  if (it != mGraphicsPipelines.end()) {
    vklLogE("you are adding existing pipeline: {}", name);
    return mGraphicsPipelines[key].get();
  }

  mGraphicsPipelines.insert({key,
                             std::make_unique<GraphicsPipeline>(name,
                                                                device,
                                                                context,
                                                                renderPass,
                                                                pipelineLayout,
                                                                subpassId)});
  return mGraphicsPipelines[key].get();
}

GraphicsPipeline* ResourcePool::requestGraphicsPipeline(const std::string& name) {
  const std::string& hash = name;

  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mGraphicsPipelines.find(key);
  if (it == mGraphicsPipelines.end()) {
    vklLogE("you are requesting none existing pipeline: {}", name);
    throw std::runtime_error("none existing pipeline");
  }

  return mGraphicsPipelines[key].get();
}

void ResourcePool::deleteGraphicsPipeline(const std::string& name) {
  const std::string&     hash = name;
  std::hash<std::string> hasher;
  size_t                 key = hasher(hash);
  auto                   it  = mGraphicsPipelines.find(key);

  if (it == mGraphicsPipelines.end()) {
    vklLogW("you are deleting none existing pipeline: {}", name);
    return;
  }

  mGraphicsPipelines.erase(key);
}
}  //namespace vkl
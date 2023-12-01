#pragma once
#include "VkLogger.h"
#include "Device.h"
#include "ShaderModule.h"
#include "VkShaderUtil.h"
#include "ResourcePool.h"
#include "Utils.h"
#include "SPIRVReflection.h"

namespace vkl {
std::unordered_map<size_t, std::unique_ptr<ShaderModule>> ResourcePool::mShaderPools;
std::unordered_map<size_t, vk::DescriptorSetLayout> ResourcePool::mDescriptorSetLayouts;

void ResourcePool::clear() {
  mShaderPools.clear();
}

ShaderModule* ResourcePool::loadShader(const std::string&      name,
                                       Device*                 device,
                                       ShaderSourceType        srcType,
                                       vk::ShaderStageFlagBits stage,
                                       std::string&            shaderSrc) {
  std::string type = stage == vk::ShaderStageFlagBits::eVertex ? "_vert_" : "_frag_";
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

}  //namespace vkl
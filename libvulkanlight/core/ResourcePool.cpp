#pragma once

#include "Device.h"
#include "ShaderModule.h"
#include "VkShaderUtil.h"
#include "ResourcePool.h"
#include "Utils.h"

namespace vkl {
std::unordered_map<size_t, std::unique_ptr<ShaderModule>> ResourcePool::shaderPools;

namespace {
struct SpirvHasher {
  uint64_t operator()(uint32_t* arr, vk::DeviceSize srcSize) const {
    uint64_t hash = 0;
    int      num  = srcSize / sizeof(uint32_t);

    for (int i = 0; i < num; i++) {
      uint32_t val = arr[i];
      hash ^= std::hash<uint32_t>{}(val) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

}  //namespace

void ResourcePool::clear() {
  shaderPools.clear();
}

ShaderModule* ResourcePool::loadShader(Device*          device,
                                       ShaderSourceType srcType,
                                       std::string&     shaderSrc) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(shaderSrc);

  bool createNew = false;
  if (shaderPools.find(key) == shaderPools.end()) {
    shaderPools.insert({key, ShaderModule::Uni(new ShaderModule(device))});
    createNew = true;
  }

  ShaderModule* out = shaderPools[key].get();
  if (!createNew) return out;

  std::vector<uint32_t> spirv;
  switch (srcType) {
  case ShaderSourceType::STRING_FILE: {
    const std::string& shaderFolderPath = Utils::getShaderPath();
    std::string        shaderFile       = shaderFolderPath + "/" + shaderSrc;
    out->vk() = VkShaderUtil::loadShader(device->getVkDevice(), shaderFile, spirv);
    break;
  }
  case ShaderSourceType::STRING: {
    vk::ShaderStageFlagBits stage = vk::ShaderStageFlagBits::eVertex;

    if (shaderSrc.find("gl_Position") == std::string::npos) {
      stage = vk::ShaderStageFlagBits::eFragment;
    }

    out->vk() = VkShaderUtil::loadShader(device->getVkDevice(), shaderSrc, stage, spirv);

    break;
  }
  case ShaderSourceType::SPV_FILE:
    break;
  case ShaderSourceType::SPV: {
    uint32_t* src  = (uint32_t*)shaderSrc.c_str();
    size_t    size = shaderSrc.length();
    out->vk()      = VkShaderUtil::loadShader(device->getVkDevice(), src, size, spirv);
    break;
  }
  default:
    break;
  }

  out->setSpirv(spirv);
  return out;
}

}  //namespace vkl
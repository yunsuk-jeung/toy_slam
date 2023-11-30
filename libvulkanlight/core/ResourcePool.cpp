#pragma once

#include "Device.h"
#include "ShaderModule.h"
#include "VkShaderUtil.h"
#include "ResourcePool.h"

namespace vkl {
std::unordered_map<uint64_t, std::unique_ptr<ShaderModule>> ResourcePool::shaderPools;

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

ShaderModule*
ResourcePool::loadShader(Device* device, uint32_t* spirv, vk::DeviceSize srcSize) {
  SpirvHasher hasher;
  uint64_t    key = hasher(spirv, srcSize);

  if (shaderPools.find(key) == shaderPools.end())
    shaderPools.insert({key, ShaderModule::Uni(new ShaderModule(device))});

  ShaderModule* out = shaderPools[key].get();

  out->vk() = VkShaderUtil::loadShader(device->getVkDevice(), spirv, srcSize);
  return out;
}
ShaderModule* ResourcePool::loadShader(Device* device, std::string& filename) {
  return nullptr;
}
ShaderModule* ResourcePool::loadShader(Device*                 device,
                                       std::string&            fileContents,
                                       vk::ShaderStageFlagBits stage) {
  return nullptr;
}
}  //namespace vkl
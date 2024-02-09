#pragma once
#include <spirv_cross/spirv_glsl.hpp>
#include <vulkan/vulkan.hpp>
#include "ShaderModule.h"
#include "VklShaderUtil.h"

namespace vkl {

class SPIRVReflection {
public:
  static void reflectShaderRresources(vk::ShaderStageFlagBits      stage,
                                      const std::vector<uint32_t>& spirv,
                                      std::vector<ShaderResource>& resources,
                                      const ShaderVariant&         variant);

protected:
  static void parseShaderResources(const spirv_cross::Compiler& compiler,
                                   vk::ShaderStageFlagBits      stage,
                                   std::vector<ShaderResource>& resources,
                                   const ShaderVariant&         variant);

  static void parsePushConstants(const spirv_cross::Compiler& compiler,
                                 vk::ShaderStageFlagBits      stage,
                                 std::vector<ShaderResource>& resources,
                                 const ShaderVariant&         variant);

  static void parseSpecializationConstants(const spirv_cross::Compiler& compiler,
                                           vk::ShaderStageFlagBits      stage,
                                           std::vector<ShaderResource>& resources,
                                           const ShaderVariant&         variant);
};
};  //namespace vkl
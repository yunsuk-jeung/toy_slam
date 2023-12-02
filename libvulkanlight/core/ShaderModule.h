#pragma once
#include "VKObject.h"
#include "Device.h"

namespace vkl {
enum class ShaderResourceType {
  Input,
  InputAttachment,
  Output,
  Image,
  ImageSampler,
  ImageStorage,
  Sampler,
  BufferUniform,
  BufferStorage,
  PushConstant,
  SpecializationConstant,
  All
};

enum class ShaderResourceMode { Static, Dynamic, UpdateAfterBind };

struct ShaderResourceQualifiers {
  enum : uint32_t {
    None        = 0,
    NonReadable = 1,
    NonWritable = 2,
  };
};

struct ShaderResource {
  vk::ShaderStageFlags stages;
  ShaderResourceType   type;
  ShaderResourceMode   mode;
  uint32_t             set;
  uint32_t             binding;
  uint32_t             location;
  uint32_t             input_attachment_index;
  uint32_t             vec_size;
  uint32_t             columns;
  uint32_t             array_size;
  uint32_t             offset;
  uint32_t             size;
  uint32_t             constant_id;
  uint32_t             qualifiers;
  std::string          name;
};

class ShaderModule : public VkObject<vk::ShaderModule> {
public:
  USING_SMART_PTR(ShaderModule);

  ShaderModule() = delete;
  ShaderModule(Device* device, vk::ShaderStageFlagBits stage)
    : mName{""}
    , mDevice{device}
    , mStage{stage} {}
  ~ShaderModule() { mDevice->vk().destroyShaderModule(this->vk()); }

protected:

protected:
  std::string                 mName;
  Device*                     mDevice;
  vk::ShaderStageFlagBits     mStage;
  std::vector<uint32_t>       mSpirv;
  std::vector<ShaderResource> mShaderResources;

  std::vector<vk::DescriptorSetLayout> mDescSetLayouts;
  std::vector<vk::PushConstantRange>   mPushConstRanges;

public:
  void                    setName(const std::string& name) { mName = name; }
  std::string&            getName() { return mName; }
  vk::ShaderStageFlagBits getStage() const { return mStage; }
  void                    setSpirv(const std::vector<uint32_t>& spirv) { mSpirv = spirv; }
  std::vector<uint32_t>&  getSpirv() { return mSpirv; }
  std::vector<ShaderResource>& getShaderResources() { return mShaderResources; }

  std::vector<vk::DescriptorSetLayout>& getDescSetLayouts() { return mDescSetLayouts; }
  std::vector<vk::PushConstantRange>&   getPushConstRanges() { return mPushConstRanges; }
};
}  //namespace vkl
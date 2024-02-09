#pragma once
#include "Device.h"
#include "Image.h"
#include "VklError.h"
#include "DescriptorSetLayout.h"
namespace vkl {
class Texture {
public:
  USING_SMART_PTR(Texture);

  Texture() = delete;
  explicit Texture(Device*           device,
                   vk::DescriptorSet descSet,
                   uint32_t          binding,
                   Image*            image,
                   vk::Sampler       sampler,
                   bool              combined)
    : mDevice{device}
    , mVkDescSet{descSet}
    , mBinding{binding}
    , mImage{image}
    , mVkSampler{sampler}
    , combined{combined} {
    updateDescSets();
  };

protected:
  void updateDescSets() {
    if (combined) {
      vk::DescriptorImageInfo imageInfo;
      imageInfo.sampler     = mVkSampler;
      imageInfo.imageView   = mImage->vkImageView();
      imageInfo.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;

      vk::WriteDescriptorSet imageWriteDescSet(mVkDescSet,
                                               mBinding,
                                               {},
                                               vk::DescriptorType::eCombinedImageSampler,
                                               imageInfo);

      mDevice->vk().updateDescriptorSets({imageWriteDescSet}, nullptr);
    }
    else {
      vk::DescriptorImageInfo samplerInfo;
      samplerInfo.sampler = mVkSampler;

      vk::WriteDescriptorSet samplerWriteDescSet(mVkDescSet,
                                                 mBinding,
                                                 {},
                                                 vk::DescriptorType::eSampler,
                                                 samplerInfo);

      vk::DescriptorImageInfo imageInfo;
      imageInfo.imageView   = mImage->vkImageView();
      imageInfo.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;

      vk::WriteDescriptorSet imageWriteDescSet(mVkDescSet,
                                               mBinding + 1u,
                                               {},
                                               vk::DescriptorType::eSampledImage,
                                               imageInfo);

      mDevice->vk().updateDescriptorSets({samplerWriteDescSet, imageWriteDescSet},
                                         nullptr);
    }
  }

protected:
  Device*           mDevice;
  vk::DescriptorSet mVkDescSet;
  uint32_t          mBinding;
  Image*            mImage;
  vk::Sampler       mVkSampler;
  bool              combined;

  //public:
  //Image* getImage() { return mImage; }
};
}  //namespace vkl
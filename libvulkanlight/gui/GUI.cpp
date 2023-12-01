#pragma once
#include <imgui.h>
#include "Utils.h"
#include "Device.h"
#include "Window.h"
#include "RenderContext.h"
#include "Buffer.h"
#include "ResourcePool.h"
#include "GUI.h"

namespace vkl {
GUI::GUI()
  : RendererBase()
  , mWindow{nullptr}
  , mPrevVBSizes{}
  , mPrevIBSizes{}
  , mVBs{}
  , mIBs{}
  , mFontImage{nullptr} {}

GUI::~GUI() {}

void GUI::onWindowResized(int w, int h) {
  auto& io         = ImGui::GetIO();
  io.DisplaySize.x = static_cast<float>(w);
  io.DisplaySize.y = static_cast<float>(h);
}

void GUI::prepare(Device*            device,
                  RenderContext*     context,
                  vk::DescriptorPool descPool,
                  vk::RenderPass     vkRenderPass,
                  std::string        pipelineName) {
  mWindow = context->getWindow();
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();

  style.WindowRounding              = 1.0f;
  style.Colors[ImGuiCol_WindowBg].w = 0.4f;

  auto const& extent         = context->getContextProps().extent;
  io.DisplaySize.x           = static_cast<float>(extent.width);
  io.DisplaySize.y           = static_cast<float>(extent.height);
  io.FontGlobalScale         = 1.0f;
  io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  mWindow->prepareGUI();

  RendererBase::prepare(device, context, descPool, vkRenderPass, pipelineName);
}

void GUI::onRender() {
  mWindow->newGUIFrame();
  ImGui::NewFrame();
  //handleInputEvents();

  ImGuiIO& io = ImGui::GetIO();
  ImGui::Begin("status");
  ImGui::Text("fps: %f", io.Framerate);

  ImGui::End();
  ImGui::Render();
}

void GUI::setName() {
  mName = "GUI";
}

void GUI::createVertexBuffer() {
  auto size = mRenderContext->getContextImageCount();
  mVBs.resize(size);

  for (auto& VB : mVBs) { VB = Buffer::Uni(new Buffer()); }

  mPrevVBSizes.resize(size, 0);
  mPrevIBSizes.resize(size, 0);
}

void GUI::createIndexBuffers() {
  auto size = mRenderContext->getContextImageCount();
  mIBs.resize(size);
  for (auto& IB : mIBs) { IB = Buffer::Uni(new Buffer()); }
}

void GUI::createTextures() {
  ImGuiIO&     io = ImGui::GetIO();
  ImFontConfig fontCf;
  fontCf.SizePixels = 30.0f;
  io.Fonts->AddFontDefault(&fontCf);

  unsigned char* pixels;
  int            width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  size_t uploadSize = width * height * 4 * sizeof(char);

  vk::Extent3D fontExtent(uint32_t(width), uint32_t(height), 1u);

  auto fontImgCI = Utils::createVkImageCI(vk::ImageType::e2D,
                                          vk::Format::eR8G8B8A8Unorm,
                                          fontExtent,
                                          vk::ImageUsageFlagBits::eSampled
                                            | vk::ImageUsageFlagBits::eTransferDst);

  auto fontImgViewCI = Utils::createVkImageViewCI(vk::ImageViewType::e2D,
                                                  vk::ImageAspectFlagBits::eColor);

  mFontImage = Image::Uni(new Image(mDevice,
                                    fontImgCI,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                                    fontImgViewCI));

  auto stagingBuffer = Buffer(mDevice,
                              uploadSize,
                              vk::BufferUsageFlagBits::eTransferSrc,
                              vk::MemoryPropertyFlagBits::eHostVisible,
                              vk::MemoryPropertyFlagBits::eHostVisible);
  stagingBuffer.update(pixels, uploadSize, 0);

  auto& renderCommandPool = mRenderContext->getRenderCommandPool();

  vk::CommandBufferAllocateInfo info(renderCommandPool,
                                     vk::CommandBufferLevel::ePrimary,
                                     1);

  auto  cmdBuffers = mDevice->vk().allocateCommandBuffers(info);
  auto& cmdBuffer  = cmdBuffers.front();

  vk::CommandBufferBeginInfo beginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  cmdBuffer.begin(beginInfo);

  cmd::setImageLayout(cmdBuffer,
                      mFontImage->vkImage,
                      vk::ImageLayout::eUndefined,
                      vk::ImageLayout::eTransferDstOptimal,
                      fontImgViewCI.subresourceRange,
                      vk::PipelineStageFlagBits::eTopOfPipe,
                      vk::PipelineStageFlagBits::eTransfer);

  vk::BufferImageCopy region;
  region.imageSubresource.aspectMask = fontImgViewCI.subresourceRange.aspectMask;
  region.imageSubresource.layerCount = fontImgViewCI.subresourceRange.layerCount;
  region.imageExtent                 = fontImgCI.extent;

  cmdBuffer.copyBufferToImage(stagingBuffer.vk(),
                              mFontImage->vkImage,
                              vk::ImageLayout::eTransferDstOptimal,
                              region);

  cmd::setImageLayout(cmdBuffer,
                      mFontImage->vkImage,
                      vk::ImageLayout::eTransferDstOptimal,
                      vk::ImageLayout::eShaderReadOnlyOptimal,
                      fontImgViewCI.subresourceRange,
                      vk::PipelineStageFlagBits::eTransfer,
                      vk::PipelineStageFlagBits::eFragmentShader);

  cmdBuffer.end();

  vk::SubmitInfo submitInfo({}, {}, cmdBuffers, {});
  mRenderContext->getQueue()->getVkQueue().submit(submitInfo);
  mDevice->vk().waitIdle();

  vk::SamplerCreateInfo samplerCI;
  samplerCI.maxAnisotropy = 1.0f;
  samplerCI.magFilter     = vk::Filter::eLinear;
  samplerCI.minFilter     = vk::Filter::eLinear;
  samplerCI.mipmapMode    = vk::SamplerMipmapMode::eLinear;
  samplerCI.addressModeU  = vk::SamplerAddressMode::eRepeat;
  samplerCI.addressModeV  = vk::SamplerAddressMode::eRepeat;
  samplerCI.addressModeW  = vk::SamplerAddressMode::eRepeat;
  samplerCI.minLod        = -1000;
  samplerCI.maxLod        = 1000;
  samplerCI.maxAnisotropy = 1.0f;

  mFontSampler = mDevice->vk().createSampler(samplerCI);
}

void GUI::createDescriptorsets() {
  auto l = ResourcePool::requestDescriptorSetLayout("imgui_frag_sTexture");

  mFontDescSet = mDevice->vk().allocateDescriptorSets({mDescPool, l}).front();

  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->SetTexID((ImTextureID)(VkDescriptorSet(mFontDescSet)));
}

}  //namespace vkl
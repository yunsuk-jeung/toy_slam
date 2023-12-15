#pragma once
#include <imgui.h>
#include "VklLogger.h"
#include "Utils.h"
#include "Device.h"
#include "Window.h"
#include "RenderContext.h"
#include "PipelineLayout.h"
#include "Pipeline.h"
#include "Buffer.h"
#include "ResourcePool.h"
#include "InputCallback.h"
#include "ImGuiObject.h"
#include "GUI.h"

namespace vkl {
GUI::GUI()
  : RendererBase()
  , mWindow{nullptr}
  , mPrevVBSizes{}
  , mPrevIBSizes{}
  , mVBs{}
  , mIBs{}
  , mFontImage{nullptr} {
  mImGuiObjects.reserve(30);
  mName = "GUI";
}

GUI::~GUI() {
  mWindow->endGUI();
  ImGui::DestroyContext();
  mDevice->vk().destroySampler(mFontSampler);
}

void GUI::onWindowResized(int w, int h) {
  auto& io         = ImGui::GetIO();
  io.DisplaySize.x = static_cast<float>(w);
  io.DisplaySize.y = static_cast<float>(h);
}

void GUI::prepare(Device*            device,
                  RenderContext*     context,
                  vk::DescriptorPool descPool,
                  vk::RenderPass     vkRenderPass,
                  Pipeline*          pipeline) {
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

  RendererBase::prepare(device, context, descPool, vkRenderPass, pipeline);
}

void GUI::addInputCallback(InputCallback* cb) {
  mInputCallbacks.push_back(cb);
}

void GUI::addImGuiObjects(std::shared_ptr<ImGui::Object> objects) {
  mImGuiObjects.push_back(objects);
}

void GUI::onRender() {
  mWindow->newGUIFrame();
  ImGui::NewFrame();
  handleInputCallbacks();

  ImGuiIO& io = ImGui::GetIO();
  ImGui::Begin("status");
  ImGui::Text("fps: %f", io.Framerate);
  ImGui::End();

  for (const auto& obj : mImGuiObjects) {
    obj->render();
  }

  ImGui::Render();
}

void GUI::buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx) {
  ImDrawData* dd = ImGui::GetDrawData();
  //idx            = 0;
  int fbWidth  = (int)(dd->DisplaySize.x * dd->FramebufferScale.x);
  int fbHeight = (int)(dd->DisplaySize.y * dd->FramebufferScale.y);
  if (fbWidth <= 0 || fbHeight <= 0)
    return;

  updateBuffer(idx);

  auto& vkPipeline = mPipeline->vk();
  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, vkPipeline);
  //Bind Vertex And Index Buffer:

  auto& VB = mVBs[idx];
  auto& IB = mIBs[idx];

  auto indexType = sizeof(ImDrawIdx) == 2 ? vk::IndexType::eUint16
                                          : vk::IndexType::eUint32;

  if (dd->TotalVtxCount > 0) {
    cmd.bindVertexBuffers(0, {VB->vk()}, {0});
    cmd.bindIndexBuffer(IB->vk(), 0, indexType);
  }

  std::vector<float> scale(2);
  scale[0] = 2.0f / dd->DisplaySize.x;
  scale[1] = 2.0f / dd->DisplaySize.y;

  std::vector<float> trans(2);
  trans[0] = -1.0f - dd->DisplayPos.x * scale[0];
  trans[1] = -1.0f - dd->DisplayPos.y * scale[1];

  auto& vkPipelineLayout = mPipelineLayout->vk();
  cmd.pushConstants<float>(vkPipelineLayout, vk::ShaderStageFlagBits::eVertex, 0, scale);
  cmd.pushConstants<float>(vkPipelineLayout, vk::ShaderStageFlagBits::eVertex, 8, trans);

  ImVec2 clipOff   = dd->DisplayPos;
  ImVec2 clipScale = dd->FramebufferScale;

  //Render command lists
  //(Because we merged all buffers into a single one, we maintain our own offset into
  //them)
  int global_vtx_offset = 0;
  int global_idx_offset = 0;
  for (int n = 0; n < dd->CmdListsCount; n++) {
    const ImDrawList* cmd_list = dd->CmdLists[n];
    for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++) {
      const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
      //Project scissor/clipping rectangles into framebuffer space
      ImVec2 clip_min((pcmd->ClipRect.x - clipOff.x) * clipScale.x,
                      (pcmd->ClipRect.y - clipOff.y) * clipScale.y);
      ImVec2 clip_max((pcmd->ClipRect.z - clipOff.x) * clipScale.x,
                      (pcmd->ClipRect.w - clipOff.y) * clipScale.y);

      //Clamp to viewport as vkCmdSetScissor() won't accept values that are off bounds
      if (clip_min.x < 0.0f) {
        clip_min.x = 0.0f;
      }
      if (clip_min.y < 0.0f) {
        clip_min.y = 0.0f;
      }
      if (clip_max.x > fbWidth) {
        clip_max.x = (float)fbWidth;
      }
      if (clip_max.y > fbHeight) {
        clip_max.y = (float)fbHeight;
      }
      if (clip_max.x <= clip_min.x || clip_max.y <= clip_min.y)
        continue;

      //Apply scissor/clipping rectangle
      VkRect2D scissor;
      scissor.offset.x      = (int32_t)(clip_min.x);
      scissor.offset.y      = (int32_t)(clip_min.y);
      scissor.extent.width  = (uint32_t)(clip_max.x - clip_min.x);
      scissor.extent.height = (uint32_t)(clip_max.y - clip_min.y);
      cmd.setScissor(0, {scissor});

      //Bind DescriptorSet with font or user texture
      vk::DescriptorSet desc_set = {(VkDescriptorSet)pcmd->TextureId};
      if (sizeof(ImTextureID) < sizeof(ImU64)) {
        //We don't support texture switches if ImTextureID hasn't been redefined to be
        //64-bit. Do a flaky check that other textures haven't been used.
        IM_ASSERT(pcmd->TextureId == mFontDescSet);
        desc_set = mFontDescSet;
      }
      cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                             vkPipelineLayout,
                             0,
                             {desc_set},
                             {});

      cmd.drawIndexed(pcmd->ElemCount,
                      1,
                      pcmd->IdxOffset + global_idx_offset,
                      pcmd->VtxOffset + global_vtx_offset,
                      0);
    }
    global_idx_offset += cmd_list->IdxBuffer.Size;
    global_vtx_offset += cmd_list->VtxBuffer.Size;
  }

  VkRect2D scissor = {
    {                0,                  0},
    {(uint32_t)fbWidth, (uint32_t)fbHeight}
  };
  cmd.setScissor(0, {scissor});
}

void GUI::updateBuffer(uint32_t idx) {
  ImDrawData* dd        = ImGui::GetDrawData();
  int         fb_width  = (int)(dd->DisplaySize.x * dd->FramebufferScale.x);
  int         fb_height = (int)(dd->DisplaySize.y * dd->FramebufferScale.y);

  if (fb_width <= 0 || fb_height <= 0)
    return;

  size_t VBSize     = dd->TotalVtxCount * sizeof(ImDrawVert);
  size_t IBSize     = dd->TotalIdxCount * sizeof(ImDrawIdx);
  auto&  prevVBSize = mPrevVBSizes[idx];
  auto&  prevIBSize = mPrevIBSizes[idx];

  if (VBSize > 0) {
    auto& VB = mVBs[idx];
    auto& IB = mIBs[idx];

    if (VBSize > prevVBSize) {
      VB.reset(new Buffer(mDevice,
                          VBSize,
                          vk::BufferUsageFlagBits::eVertexBuffer,
                          vk::MemoryPropertyFlagBits::eHostVisible,
                          vk::MemoryPropertyFlagBits::eHostCoherent));
    }
    if (IBSize > prevIBSize) {
      IB.reset(new Buffer(mDevice,
                          IBSize,
                          vk::BufferUsageFlagBits::eIndexBuffer,
                          vk::MemoryPropertyFlagBits::eHostVisible,
                          vk::MemoryPropertyFlagBits::eHostCoherent));
    }
    ImDrawVert* pVBO = (ImDrawVert*)VB->map();
    ImDrawIdx*  pIBO = (ImDrawIdx*)IB->map();

    for (int n = 0; n < dd->CmdListsCount; n++) {
      const ImDrawList* cmd_list = dd->CmdLists[n];
      memcpy(pVBO,
             cmd_list->VtxBuffer.Data,
             cmd_list->VtxBuffer.Size * sizeof(ImDrawVert));
      memcpy(pIBO,
             cmd_list->IdxBuffer.Data,
             cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx));
      pVBO += cmd_list->VtxBuffer.Size;
      pIBO += cmd_list->IdxBuffer.Size;
    }

    VB->flush();
    IB->flush();
    VB->unmap();
    IB->unmap();

    prevVBSize = VBSize;
    prevIBSize = IBSize;
  }
}

void GUI::handleInputCallbacks() {
  ImGuiIO& io = ImGui::GetIO();

  if (!io.WantCaptureKeyboard) {
    ImGuiKey key_first = (ImGuiKey)0;
    ImGuiKey key       = key_first;
    struct funcs {
      static bool IsLegacyNativeDupe(ImGuiKey key) {
        return key < 512 && ImGui::GetIO().KeyMap[key] != -1;
      }
    };  //Hide Native<>ImGuiKey duplicates when both exists in the array

    for (; key < ImGuiKey_COUNT; key = (ImGuiKey)(key + 1)) {
      if (funcs::IsLegacyNativeDupe(key))
        continue;

      if (ImGui::IsKeyDown(key)) {
        //VklLogI("key  is down");
      }
      if (ImGui::IsKeyReleased(key)) {
        ImVec2 pos = ImGui::GetCursorScreenPos();
      }

      if (ImGui::IsKeyPressed(key)) {
        for (auto& cb : mInputCallbacks) {
          (*cb).onKeyPressed(key);
        }
      }
    }
  }

  if (!io.WantCaptureMouse) {
    auto mpos = io.MousePos;
    for (int i = 0; i < 3; i++) {
      if (ImGui::IsMouseClicked(i)) {
        for (auto& cb : mInputCallbacks) {
          (*cb).onMouseClick(i, mpos.x, mpos.y);
        }
      }
      if (ImGui::IsMouseDragging(i)) {
        for (auto& cb : mInputCallbacks) {
          //(*cb).onMouseDrag(i, io.MousePos.x, io.MousePos.y);
          (*cb).onMouseDrag(i, mpos.x, mpos.y);
        }
      }

      if (io.MouseWheel) {
        for (auto& cb : mInputCallbacks) {
          (*cb).onMouseWheel(io.MouseWheel);
        }
      }
    }  //0 left //1 right //2 wheel
  };
}

void GUI::createVertexBuffer() {
  auto size = mRenderContext->getContextImageCount();
  mVBs.resize(size);

  for (auto& VB : mVBs) {
    VB = std::make_unique<Buffer>();
  }

  mPrevVBSizes.resize(size, 0);
  mPrevIBSizes.resize(size, 0);
}

void GUI::createIndexBuffers() {
  auto size = mRenderContext->getContextImageCount();
  mIBs.resize(size);
  for (auto& IB : mIBs) {
    IB = std::make_unique<Buffer>();
  }
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

  mFontDescSet = mDevice->vk().allocateDescriptorSets({mVkDescPool, l}).front();

  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->SetTexID((ImTextureID)(VkDescriptorSet(mFontDescSet)));
}

void GUI::updateDescriptorsets() {
  vk::DescriptorImageInfo fontDescImageInfo(mFontSampler,
                                            mFontImage->vkImageView,
                                            vk::ImageLayout::eShaderReadOnlyOptimal);

  vk::WriteDescriptorSet fontWriteDescSet(mFontDescSet,
                                          0,
                                          {},
                                          vk::DescriptorType::eCombinedImageSampler,
                                          fontDescImageInfo);

  mDevice->vk().updateDescriptorSets(fontWriteDescSet, {});
}

}  //namespace vkl
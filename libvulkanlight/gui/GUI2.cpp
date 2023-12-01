#include <iostream>
#include <imgui.h>
#include "GUI2.h"
#include "Device.h"
#include "RenderContext.h"
#include "Buffer.h"
#include "InputCallback.h"
#include "window/Window.h"
#include "Utils.h"
#include "VkShaderUtil.h"
#include "VkLogger.h"
#include "VkError.h"
#include "ShaderModule.h"

namespace vkl {

GUI2::GUI2() {
  setName();
  setShader();
}

GUI2::~GUI2() {
  window->endGUI();
  ImGui::DestroyContext();

  device->vk().destroySampler(fontSampler);
}

void GUI2::onWindowResized(int w, int h) {
  auto& io         = ImGui::GetIO();
  io.DisplaySize.x = static_cast<float>(w);
  io.DisplaySize.y = static_cast<float>(h);
}

void GUI2::initialize(Device*            _device,
                     RenderContext*     context,
                     vk::DescriptorPool descPool) {
  VkBaseRenderer::initialize(_device, context, descPool);

  guiCmdBuffer = renderContext->getOneTimeCommandBuffer();

  window = renderContext->getWindow();
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();

  style.WindowRounding              = 1.0f;
  style.Colors[ImGuiCol_WindowBg].w = 0.4f;

  auto const& extent         = renderContext->getContextProps().extent;
  io.DisplaySize.x           = static_cast<float>(extent.width);
  io.DisplaySize.y           = static_cast<float>(extent.height);
  io.FontGlobalScale         = 1.0f;
  io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  window->prepareGUI();
}

void GUI2::buildCommandBuffer(vk::CommandBuffer cmd, uint32_t idx) {
  ImDrawData* dd = ImGui::GetDrawData();
  //idx            = 0;
  int fbWidth  = (int)(dd->DisplaySize.x * dd->FramebufferScale.x);
  int fbHeight = (int)(dd->DisplaySize.y * dd->FramebufferScale.y);
  if (fbWidth <= 0 || fbHeight <= 0) return;

  updateImGuiBuffer(idx);

  cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, vkPipeline);
  //Bind Vertex And Index Buffer:

  auto& VB = VBs[idx];
  auto& IB = IBs[idx];

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
      if (clip_min.x < 0.0f) { clip_min.x = 0.0f; }
      if (clip_min.y < 0.0f) { clip_min.y = 0.0f; }
      if (clip_max.x > fbWidth) { clip_max.x = (float)fbWidth; }
      if (clip_max.y > fbHeight) { clip_max.y = (float)fbHeight; }
      if (clip_max.x <= clip_min.x || clip_max.y <= clip_min.y) continue;

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
        IM_ASSERT(pcmd->TextureId == fontDescSet);
        desc_set = fontDescSet;
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

void GUI2::onRender() {
  window->newGUIFrame();
  ImGui::NewFrame();
  handleInputEvents();

  ImGuiIO& io = ImGui::GetIO();
  ImGui::Begin("test");
  ImGui::Text("fps: %f", io.Framerate);

  ImGui::End();
  ImGui::Render();
}

void GUI2::addInputCallback(InputCallback* cb) {
  inputCallbacks.push_back(cb);
}

void GUI2::handleInputEvents() {
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
      if (funcs::IsLegacyNativeDupe(key)) continue;

      if (ImGui::IsKeyDown(key)) {
        //VklLogI("key  is down");
      }
      if (ImGui::IsKeyReleased(key)) { ImVec2 pos = ImGui::GetCursorScreenPos(); }

      if (ImGui::IsKeyPressed(key)) {
        for (auto& cb : inputCallbacks) { (*cb).onKeyPressed(key); }
      }
    }
  }

  if (!io.WantCaptureMouse) {
    auto mpos = io.MousePos;
    for (int i = 0; i < 3; i++) {
      if (ImGui::IsMouseClicked(i)) {
        for (auto& cb : inputCallbacks) { (*cb).onMouseClick(i, mpos.x, mpos.y); }
      }
      if (ImGui::IsMouseDragging(i)) {
        for (auto& cb : inputCallbacks) {
          //(*cb).onMouseDrag(i, io.MousePos.x, io.MousePos.y);
          (*cb).onMouseDrag(i, mpos.x, mpos.y);
        }
      }

      if (io.MouseWheel) {
        for (auto& cb : inputCallbacks) { (*cb).onMouseWheel(io.MouseWheel); }
      }
    }  //0 left //1 right //2 wheel
  };
}

void GUI2::setName() {
  name = "VkTriangleWithTexture Renderer";
}

void GUI2::setShader() {
  shaderName = "imgui";
  //glsl_shader.vert, compiled with:
  //# glslangValidator -V -x -o glsl_shader.vert.u32 glsl_shader.vert
  /*
  #version 450 core
  layout(location = 0) in vec2 aPos;
  layout(location = 1) in vec2 aUV;
  layout(location = 2) in vec4 aColor;
  layout(push_constant) uniform uPushConstant { vec2 uScale; vec2 uTranslate; } pc;

  out gl_PerVertex { vec4 gl_Position; };
  layout(location = 0) out struct { vec4 Color; vec2 UV; } Out;

  void main()
  {
      Out.Color = aColor;
      Out.UV = aUV;
      gl_Position = vec4(aPos * pc.uScale + pc.uTranslate, 0, 1);
  }
  */
  uint32_t vertspv[] =
    {0x07230203, 0x00010000, 0x00080001, 0x0000002e, 0x00000000, 0x00020011, 0x00000001,
     0x0006000b, 0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e,
     0x00000000, 0x00000001, 0x000a000f, 0x00000000, 0x00000004, 0x6e69616d, 0x00000000,
     0x0000000b, 0x0000000f, 0x00000015, 0x0000001b, 0x0000001c, 0x00030003, 0x00000002,
     0x000001c2, 0x00040005, 0x00000004, 0x6e69616d, 0x00000000, 0x00030005, 0x00000009,
     0x00000000, 0x00050006, 0x00000009, 0x00000000, 0x6f6c6f43, 0x00000072, 0x00040006,
     0x00000009, 0x00000001, 0x00005655, 0x00030005, 0x0000000b, 0x0074754f, 0x00040005,
     0x0000000f, 0x6c6f4361, 0x0000726f, 0x00030005, 0x00000015, 0x00565561, 0x00060005,
     0x00000019, 0x505f6c67, 0x65567265, 0x78657472, 0x00000000, 0x00060006, 0x00000019,
     0x00000000, 0x505f6c67, 0x7469736f, 0x006e6f69, 0x00030005, 0x0000001b, 0x00000000,
     0x00040005, 0x0000001c, 0x736f5061, 0x00000000, 0x00060005, 0x0000001e, 0x73755075,
     0x6e6f4368, 0x6e617473, 0x00000074, 0x00050006, 0x0000001e, 0x00000000, 0x61635375,
     0x0000656c, 0x00060006, 0x0000001e, 0x00000001, 0x61725475, 0x616c736e, 0x00006574,
     0x00030005, 0x00000020, 0x00006370, 0x00040047, 0x0000000b, 0x0000001e, 0x00000000,
     0x00040047, 0x0000000f, 0x0000001e, 0x00000002, 0x00040047, 0x00000015, 0x0000001e,
     0x00000001, 0x00050048, 0x00000019, 0x00000000, 0x0000000b, 0x00000000, 0x00030047,
     0x00000019, 0x00000002, 0x00040047, 0x0000001c, 0x0000001e, 0x00000000, 0x00050048,
     0x0000001e, 0x00000000, 0x00000023, 0x00000000, 0x00050048, 0x0000001e, 0x00000001,
     0x00000023, 0x00000008, 0x00030047, 0x0000001e, 0x00000002, 0x00020013, 0x00000002,
     0x00030021, 0x00000003, 0x00000002, 0x00030016, 0x00000006, 0x00000020, 0x00040017,
     0x00000007, 0x00000006, 0x00000004, 0x00040017, 0x00000008, 0x00000006, 0x00000002,
     0x0004001e, 0x00000009, 0x00000007, 0x00000008, 0x00040020, 0x0000000a, 0x00000003,
     0x00000009, 0x0004003b, 0x0000000a, 0x0000000b, 0x00000003, 0x00040015, 0x0000000c,
     0x00000020, 0x00000001, 0x0004002b, 0x0000000c, 0x0000000d, 0x00000000, 0x00040020,
     0x0000000e, 0x00000001, 0x00000007, 0x0004003b, 0x0000000e, 0x0000000f, 0x00000001,
     0x00040020, 0x00000011, 0x00000003, 0x00000007, 0x0004002b, 0x0000000c, 0x00000013,
     0x00000001, 0x00040020, 0x00000014, 0x00000001, 0x00000008, 0x0004003b, 0x00000014,
     0x00000015, 0x00000001, 0x00040020, 0x00000017, 0x00000003, 0x00000008, 0x0003001e,
     0x00000019, 0x00000007, 0x00040020, 0x0000001a, 0x00000003, 0x00000019, 0x0004003b,
     0x0000001a, 0x0000001b, 0x00000003, 0x0004003b, 0x00000014, 0x0000001c, 0x00000001,
     0x0004001e, 0x0000001e, 0x00000008, 0x00000008, 0x00040020, 0x0000001f, 0x00000009,
     0x0000001e, 0x0004003b, 0x0000001f, 0x00000020, 0x00000009, 0x00040020, 0x00000021,
     0x00000009, 0x00000008, 0x0004002b, 0x00000006, 0x00000028, 0x00000000, 0x0004002b,
     0x00000006, 0x00000029, 0x3f800000, 0x00050036, 0x00000002, 0x00000004, 0x00000000,
     0x00000003, 0x000200f8, 0x00000005, 0x0004003d, 0x00000007, 0x00000010, 0x0000000f,
     0x00050041, 0x00000011, 0x00000012, 0x0000000b, 0x0000000d, 0x0003003e, 0x00000012,
     0x00000010, 0x0004003d, 0x00000008, 0x00000016, 0x00000015, 0x00050041, 0x00000017,
     0x00000018, 0x0000000b, 0x00000013, 0x0003003e, 0x00000018, 0x00000016, 0x0004003d,
     0x00000008, 0x0000001d, 0x0000001c, 0x00050041, 0x00000021, 0x00000022, 0x00000020,
     0x0000000d, 0x0004003d, 0x00000008, 0x00000023, 0x00000022, 0x00050085, 0x00000008,
     0x00000024, 0x0000001d, 0x00000023, 0x00050041, 0x00000021, 0x00000025, 0x00000020,
     0x00000013, 0x0004003d, 0x00000008, 0x00000026, 0x00000025, 0x00050081, 0x00000008,
     0x00000027, 0x00000024, 0x00000026, 0x00050051, 0x00000006, 0x0000002a, 0x00000027,
     0x00000000, 0x00050051, 0x00000006, 0x0000002b, 0x00000027, 0x00000001, 0x00070050,
     0x00000007, 0x0000002c, 0x0000002a, 0x0000002b, 0x00000028, 0x00000029, 0x00050041,
     0x00000011, 0x0000002d, 0x0000001b, 0x0000000d, 0x0003003e, 0x0000002d, 0x0000002c,
     0x000100fd, 0x00010038};

  //glsl_shader.frag, compiled with:
  //# glslangValidator -V -x -o glsl_shader.frag.u32 glsl_shader.frag
  /*
  #version 450 core
  layout(location = 0) out vec4 fColor;
  layout(set=0, binding=0) uniform sampler2D sTexture;
  layout(location = 0) in struct { vec4 Color; vec2 UV; } In;
  void main()
  {
      fColor = In.Color * texture(sTexture, In.UV.st);
  }
  */
  uint32_t fragspv[] =
    {0x07230203, 0x00010000, 0x00080001, 0x0000001e, 0x00000000, 0x00020011, 0x00000001,
     0x0006000b, 0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e,
     0x00000000, 0x00000001, 0x0007000f, 0x00000004, 0x00000004, 0x6e69616d, 0x00000000,
     0x00000009, 0x0000000d, 0x00030010, 0x00000004, 0x00000007, 0x00030003, 0x00000002,
     0x000001c2, 0x00040005, 0x00000004, 0x6e69616d, 0x00000000, 0x00040005, 0x00000009,
     0x6c6f4366, 0x0000726f, 0x00030005, 0x0000000b, 0x00000000, 0x00050006, 0x0000000b,
     0x00000000, 0x6f6c6f43, 0x00000072, 0x00040006, 0x0000000b, 0x00000001, 0x00005655,
     0x00030005, 0x0000000d, 0x00006e49, 0x00050005, 0x00000016, 0x78655473, 0x65727574,
     0x00000000, 0x00040047, 0x00000009, 0x0000001e, 0x00000000, 0x00040047, 0x0000000d,
     0x0000001e, 0x00000000, 0x00040047, 0x00000016, 0x00000022, 0x00000000, 0x00040047,
     0x00000016, 0x00000021, 0x00000000, 0x00020013, 0x00000002, 0x00030021, 0x00000003,
     0x00000002, 0x00030016, 0x00000006, 0x00000020, 0x00040017, 0x00000007, 0x00000006,
     0x00000004, 0x00040020, 0x00000008, 0x00000003, 0x00000007, 0x0004003b, 0x00000008,
     0x00000009, 0x00000003, 0x00040017, 0x0000000a, 0x00000006, 0x00000002, 0x0004001e,
     0x0000000b, 0x00000007, 0x0000000a, 0x00040020, 0x0000000c, 0x00000001, 0x0000000b,
     0x0004003b, 0x0000000c, 0x0000000d, 0x00000001, 0x00040015, 0x0000000e, 0x00000020,
     0x00000001, 0x0004002b, 0x0000000e, 0x0000000f, 0x00000000, 0x00040020, 0x00000010,
     0x00000001, 0x00000007, 0x00090019, 0x00000013, 0x00000006, 0x00000001, 0x00000000,
     0x00000000, 0x00000000, 0x00000001, 0x00000000, 0x0003001b, 0x00000014, 0x00000013,
     0x00040020, 0x00000015, 0x00000000, 0x00000014, 0x0004003b, 0x00000015, 0x00000016,
     0x00000000, 0x0004002b, 0x0000000e, 0x00000018, 0x00000001, 0x00040020, 0x00000019,
     0x00000001, 0x0000000a, 0x00050036, 0x00000002, 0x00000004, 0x00000000, 0x00000003,
     0x000200f8, 0x00000005, 0x00050041, 0x00000010, 0x00000011, 0x0000000d, 0x0000000f,
     0x0004003d, 0x00000007, 0x00000012, 0x00000011, 0x0004003d, 0x00000014, 0x00000017,
     0x00000016, 0x00050041, 0x00000019, 0x0000001a, 0x0000000d, 0x00000018, 0x0004003d,
     0x0000000a, 0x0000001b, 0x0000001a, 0x00050057, 0x00000007, 0x0000001c, 0x00000017,
     0x0000001b, 0x00050085, 0x00000007, 0x0000001d, 0x00000012, 0x0000001c, 0x0003003e,
     0x00000009, 0x0000001d, 0x000100fd, 0x00010038};

  shaderSrcType    = ShaderSourceType::SPV;
  vertShaderSource = std::string((const char*)vertspv, sizeof(vertspv));
  fragShaderSource = std::string((const char*)fragspv, sizeof(fragspv));

  uint32_t sample = vertspv[0];
}

void GUI2::createVkDescriptorSetLayout() {
  std::array<vk::DescriptorSetLayoutBinding, 1> bindings = {
    {{0,
      vk::DescriptorType::eCombinedImageSampler,
      1,
      vk::ShaderStageFlagBits::eFragment}}};

  texDescSetLayout = device->vk().createDescriptorSetLayout({{}, bindings});
  createdDescriptorSetLayouts.push(texDescSetLayout);
}

void GUI2::createVkDescriptorSets() {
  fontDescSet = device->vk()
                  .allocateDescriptorSets({vkDescriptorPool, texDescSetLayout})
                  .front();

  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->SetTexID((ImTextureID)(VkDescriptorSet(fontDescSet)));
}

void GUI2::updateDescriptorSets() {
  vk::DescriptorImageInfo fontDescImageInfo(fontSampler,
                                            fontImage.vkImageView,
                                            vk::ImageLayout::eShaderReadOnlyOptimal);

  vk::WriteDescriptorSet fontWriteDescSet(fontDescSet,
                                          0,
                                          {},
                                          vk::DescriptorType::eCombinedImageSampler,
                                          fontDescImageInfo);

  device->vk().updateDescriptorSets(fontWriteDescSet, {});

  auto count = renderContext->getContextImageCount();
}

void GUI2::createVkPipelineLayout() {
  vk::PushConstantRange pushConst;
  pushConst.stageFlags = vk::ShaderStageFlagBits::eVertex;
  pushConst.offset     = 0;
  pushConst.size       = sizeof(float) * 4;

  vk::PipelineLayoutCreateInfo pipelineLayoutCI({}, texDescSetLayout, pushConst);

  vkPipelineLayout = device->vk().createPipelineLayout(pipelineLayoutCI);
}

void GUI2::createVkPipeline(vk::RenderPass renderPass) {
  if (vkPipeline) { device->vk().destroyPipeline(vkPipeline); }

  std::vector<vk::PipelineShaderStageCreateInfo> shaderStageCIs{
    {{},   vk::ShaderStageFlagBits::eVertex, vertShader->vk(), "main"},
    {{}, vk::ShaderStageFlagBits::eFragment, fragShader->vk(), "main"}
  };

  std::vector<vk::VertexInputBindingDescription> vertBindingDescription{
    {0, sizeof(ImDrawVert), vk::VertexInputRate::eVertex}
  };

  std::vector<vk::VertexInputAttributeDescription> attributeDescription{
    {0, 0,  vk::Format::eR32G32Sfloat, IM_OFFSETOF(ImDrawVert, pos)},
    {1, 0,  vk::Format::eR32G32Sfloat, IM_OFFSETOF(ImDrawVert,  uv)},
    {2, 0, vk::Format::eR8G8B8A8Unorm, IM_OFFSETOF(ImDrawVert, col)}
  };

  vk::PipelineVertexInputStateCreateInfo inputStateCI({},
                                                      vertBindingDescription,
                                                      attributeDescription);

  vk::PipelineInputAssemblyStateCreateInfo
    inputAssemCI({}, vk::PrimitiveTopology::eTriangleList, false);

  vk::PipelineViewportStateCreateInfo viewport_state({}, 1, nullptr, 1, nullptr);

  vk::PipelineRasterizationStateCreateInfo rasterizationState;
  rasterizationState.polygonMode = vk::PolygonMode::eFill;
  rasterizationState.cullMode    = vk::CullModeFlagBits::eNone;
  rasterizationState.frontFace   = vk::FrontFace::eClockwise;
  rasterizationState.lineWidth   = 1.0f;

  vk::PipelineMultisampleStateCreateInfo multisample_state({},
                                                           vk::SampleCountFlagBits::e1);
  vk::PipelineColorBlendAttachmentState  blendAttachmentState;
  blendAttachmentState.blendEnable         = true;
  blendAttachmentState.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
  blendAttachmentState.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
  blendAttachmentState.colorBlendOp        = vk::BlendOp::eAdd;
  blendAttachmentState.srcAlphaBlendFactor = vk::BlendFactor::eOne;
  blendAttachmentState.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
  blendAttachmentState.alphaBlendOp        = vk::BlendOp::eAdd;
  blendAttachmentState.colorWriteMask      = vk::ColorComponentFlagBits::eR
                                        | vk::ColorComponentFlagBits::eG
                                        | vk::ColorComponentFlagBits::eB
                                        | vk::ColorComponentFlagBits::eA;

  vk::PipelineDepthStencilStateCreateInfo depthStencilState;
  depthStencilState.depthCompareOp   = vk::CompareOp::eAlways;
  depthStencilState.depthTestEnable  = true;
  depthStencilState.depthWriteEnable = true;
  depthStencilState.back.compareOp   = vk::CompareOp::eAlways;
  depthStencilState.front            = depthStencilState.back;

  vk::PipelineColorBlendStateCreateInfo colorBlendState({}, {}, {}, blendAttachmentState);

  std::array<vk::DynamicState, 2> dynamicStateEnables = {vk::DynamicState::eViewport,
                                                         vk::DynamicState::eScissor};

  vk::PipelineDynamicStateCreateInfo dynamicState({}, dynamicStateEnables);

  vk::GraphicsPipelineCreateInfo pipelineCI({},
                                            shaderStageCIs,
                                            &inputStateCI,
                                            &inputAssemCI,
                                            {},
                                            &viewport_state,
                                            &rasterizationState,
                                            &multisample_state,
                                            &depthStencilState,
                                            &colorBlendState,
                                            &dynamicState,
                                            vkPipelineLayout,
                                            renderPass,
                                            subpassId);

  vk::Result result;
  std::tie(result, vkPipeline) = device->vk().createGraphicsPipeline(VK_NULL_HANDLE,
                                                                     pipelineCI);
  VK_CHECK_ERROR(static_cast<VkResult>(result), "createpipeline");
}

void GUI2::createVertexBuffers() {
  auto size = renderContext->getContextImageCount();
  VBs.resize(size);

  for (auto& VB : VBs) { VB = Buffer::Uni(new Buffer); }

  prevVBSizes.resize(size, 0);
  prevIBSizes.resize(size, 0);
}

void GUI2::createIndexBuffers() {
  auto size = renderContext->getContextImageCount();
  IBs.resize(size);
  for (auto& IB : IBs) { IB = Buffer::Uni(new Buffer); }
}

void GUI2::createUniformBuffers() {}

void GUI2::createTextures() {
  ImGuiIO& io = ImGui::GetIO();
  //io.Fonts->AddFontFromFileTTF("./Fira.ttf", 18.0f);
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

  fontImage = Image(device,
                    fontImgCI,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    vk::MemoryPropertyFlagBits::eDeviceLocal,
                    fontImgViewCI);

  auto stagingBuffer = Buffer(device,
                              uploadSize,
                              vk::BufferUsageFlagBits::eTransferSrc,
                              vk::MemoryPropertyFlagBits::eHostVisible,
                              vk::MemoryPropertyFlagBits::eHostVisible);
  stagingBuffer.update(pixels, uploadSize, 0);

  auto& renderCommandPool = renderContext->getRenderCommandPool();

  vk::CommandBufferAllocateInfo info(renderCommandPool,
                                     vk::CommandBufferLevel::ePrimary,
                                     1);

  auto  cmdBuffers = device->vk().allocateCommandBuffers(info);
  auto& cmdBuffer  = cmdBuffers.front();

  vk::CommandBufferBeginInfo beginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);

  cmdBuffer.begin(beginInfo);

  cmd::setImageLayout(cmdBuffer,
                      fontImage.vkImage,
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
                              fontImage.vkImage,
                              vk::ImageLayout::eTransferDstOptimal,
                              region);

  cmd::setImageLayout(cmdBuffer,
                      fontImage.vkImage,
                      vk::ImageLayout::eTransferDstOptimal,
                      vk::ImageLayout::eShaderReadOnlyOptimal,
                      fontImgViewCI.subresourceRange,
                      vk::PipelineStageFlagBits::eTransfer,
                      vk::PipelineStageFlagBits::eFragmentShader);

  cmdBuffer.end();

  vk::SubmitInfo submitInfo({}, {}, cmdBuffers, {});
  renderContext->getQueue()->getVkQueue().submit(submitInfo);
  device->vk().waitIdle();

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

  fontSampler = device->vk().createSampler(samplerCI);
}

void GUI2::updateImGuiBuffer(uint32_t idx) {
  ImDrawData* dd        = ImGui::GetDrawData();
  int         fb_width  = (int)(dd->DisplaySize.x * dd->FramebufferScale.x);
  int         fb_height = (int)(dd->DisplaySize.y * dd->FramebufferScale.y);
  if (fb_width <= 0 || fb_height <= 0) return;

  size_t VBSize     = dd->TotalVtxCount * sizeof(ImDrawVert);
  size_t IBSize     = dd->TotalIdxCount * sizeof(ImDrawIdx);
  auto&  prevVBSize = prevVBSizes[idx];
  auto&  prevIBSize = prevIBSizes[idx];

  if (VBSize > 0) {
    auto& VB = VBs[idx];
    auto& IB = IBs[idx];

    if (VBSize > prevVBSize) {
      VB = Buffer::Uni(new Buffer(device,
                                  VBSize,
                                  vk::BufferUsageFlagBits::eVertexBuffer,
                                  vk::MemoryPropertyFlagBits::eHostVisible,
                                  vk::MemoryPropertyFlagBits::eHostCoherent));
    }
    if (IBSize > prevIBSize) {
      IB = Buffer::Uni(new Buffer(device,
                                  IBSize,
                                  vk::BufferUsageFlagBits::eIndexBuffer,
                                  vk::MemoryPropertyFlagBits::eHostVisible,
                                  vk::MemoryPropertyFlagBits::eHostCoherent));
    }
    uint8_t* pVBO = VB->map();
    uint8_t* pIBO = IB->map();

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
}  //namespace vkl

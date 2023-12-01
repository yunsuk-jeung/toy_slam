#pragma once
#include <imgui.h>
#include "Window.h"
#include "RenderContext.h"
#include "Buffer.h"
#include "GUI.h"

namespace vkl {
GUI::GUI()
  : RendererBase()
  , mWindow{nullptr} {}

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
  RendererBase::prepare(device, context, descPool, vkRenderPass, pipelineName);

  mWindow = context->getWindow();
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();

  style.WindowRounding              = 1.0f;
  style.Colors[ImGuiCol_WindowBg].w = 0.4f;

  auto const& extent         = mRenderContext->getContextProps().extent;
  io.DisplaySize.x           = static_cast<float>(extent.width);
  io.DisplaySize.y           = static_cast<float>(extent.height);
  io.FontGlobalScale         = 1.0f;
  io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  mWindow->prepareGUI();
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

}  //namespace vkl
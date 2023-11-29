#pragma once

#include <string>
#include <memory>
#include <volk.h>
#include <vulkan/vulkan.hpp>
#include "core/Buffer.h"
#include "shaders/ShaderTypes.h"
#include "types.h"

namespace vkl {
class InputCallback;
class GUI;
class Instance;
class Window;
class Device;
class RenderContext;
class GraphicsCamera;
class Application {
public:
  Application();
  virtual ~Application();

  void setShaderPath(const std::string& shaderPath);
  void setResourcePath(const std::string& shaderPath);

  void registerWindow(std::unique_ptr<Window>& _window);

  virtual void onWindowResized(int w, int h, int orientation = 0);
  virtual bool prepare();
  virtual void run();
  virtual void end();

  virtual void onRender();

protected:
  void         createInstance();
  void         createDevice();
  void         createRenderContext();
  virtual void createCommandPool();
  virtual void createCommandBuffer();
  virtual void createRenderPass();
  virtual void createFrameBuffer();
  virtual void createVkDescriptorPool();
  virtual void createGUI();
  virtual void createGraphicsCamera();

  void requestGpuFeatures(Device* vkPhysicalDevice);

  virtual void              buildCommandBuffer();
  virtual vk::CommandBuffer beginCommandBuffer();
  virtual void              beginRenderPass(vk::CommandBuffer cmd);
  virtual void              updateCameraUniform(int idx);
  virtual void              updateUniform(int idx);

  void prepareFrame();
  void presentFrame();

protected:
  std::string name           = "Default Application";
  uint32_t    apiVersion     = VK_API_VERSION_1_0;
  bool        prepared       = false;
  bool        endApplication = true;

  std::unique_ptr<Instance>      instance;
  std::unique_ptr<Window>        window;
  vk::SurfaceKHR                 vkSurface;
  std::unique_ptr<Device>        device;
  std::unique_ptr<RenderContext> renderContext;
  vk::RenderPass                 vkRenderPass;
  std::vector<vk::Framebuffer>   vkFramebuffers;
  vk::DescriptorPool             vkDescPool;

  vk::Fence          currCmdFence;
  BufferingSemaphore currScSemaphore;
  uint32_t           currSwapchainIdx;

  uint32_t                       lastSubpass;
  std::unique_ptr<GUI>           gui;
  std::unique_ptr<InputCallback> inputCallback;

  //used for camera
  std::unique_ptr<GraphicsCamera> graphicsCamera;

  std::vector<Buffer> camUBs;
  //vk::DescriptorPool             camDescPool = VK_NULL_HANDLE;
  vk::DescriptorSetLayout        camDescLayout;
  std::vector<vk::DescriptorSet> camDescSets;

public:
  Device*        getDevice() { return device.get(); }
  RenderContext* getRenderContext() { return renderContext.get(); }
  //std::vector<const char*> intanceExtensions;
  //std::vector<const char*> validationLayers;
};
}  //namespace vkl
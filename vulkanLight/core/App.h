#pragma once

#include <memory>
#include <vulkan/vulkan.hpp>

#include "types.h"
#include "ShaderTypes.h"

namespace vkl {
class InputCallback;
class GUI;
class Instance;
class Window;
class Device;
class RenderContext;
class GraphicsCamera;
class App {
public:
  App();
  virtual ~App();

  void registerWindow(std::unique_ptr<Window>& _window);

  virtual void onWindowResized(int w, int h);
  virtual bool prepare();
  virtual void run();
  virtual void end();

protected:
  void         createInstance();
  void         createDevice();
  void         createRenderContext();
  virtual void createCommandPool();
  virtual void createCommandBuffer();
  virtual void initializeSwapchainFrames();
  virtual void createRenderPass();
  virtual void createFrameBuffer();
  virtual void createGUI();
  virtual void createGraphicsCamera();
  virtual void createVkDescriptorPool();

  void requestGpuFeatures(Device* vkPhysicalDevice);

  virtual void              buildCommandBuffer();
  virtual vk::CommandBuffer beginCommandBuffer();
  virtual void              updateCameraUniform(int idx);
  virtual void              updateUniform(int idx);
  virtual void              onRender();

  void prepareFrame();
  void presentFrame();

protected:
  std::string name       = "Default Application";
  uint32_t    apiVersion = VK_API_VERSION_1_0;
  bool        prepared   = false;
  bool        endApp     = false;

  std::unique_ptr<Instance>      instance      = nullptr;
  std::unique_ptr<Window>        window        = nullptr;
  vk::SurfaceKHR                 vkSurface     = nullptr;
  std::unique_ptr<Device>        device        = nullptr;
  std::unique_ptr<RenderContext> renderContext = nullptr;
  vk::RenderPass                 vkRenderPass  = nullptr;
  std::vector<vk::Framebuffer>   vkFramebuffers;

  vk::DescriptorPool vkDescPool = VK_NULL_HANDLE;

  vk::Fence          currCmdFence    = VK_NULL_HANDLE;
  BufferingSemaphore currScSemaphore = {VK_NULL_HANDLE, VK_NULL_HANDLE};
  uint32_t           currScIdx       = 0;

  std::unique_ptr<GUI>           gui           = nullptr;
  std::unique_ptr<InputCallback> inputCallback = nullptr;

  //used for camera, camera is dependent on gui
  std::unique_ptr<GraphicsCamera> graphicsCamera = nullptr;
  std::unique_ptr<Uniform>        camUniform     = nullptr;

public:
  Device*        getDevice() { return device.get(); }
  RenderContext* getRenderContext() { return renderContext.get(); }
};
}  //namespace vkl
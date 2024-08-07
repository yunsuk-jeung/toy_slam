#pragma once
#include <memory>
#include <string>
#include <volk.h>
#include <vulkan/vulkan.hpp>
#include "Buffer.h"
#include "ShaderTypes.h"
#include "vkltypes.h"

namespace vkl {
class InputCallback;
class GUI;
class Instance;
class Window;
class Device;
class RenderContext;
class GraphicsCamera;
class UniformBuffer;
class DescriptorSetLayout;
class App {
public:
  App();
  virtual ~App();

  void addShaderPath(const std::string& shaderPath);
  void addAssetPath(const std::string& assetPath);

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
  virtual void createPipelines();
  virtual void createRenderers();
  virtual void createGUI();
  virtual void createGraphicsCamera();

  void requestGpuFeatures(Device* vkPhysicalDevice);

  virtual void              buildCommandBuffer();
  virtual vk::CommandBuffer beginCommandBuffer();
  virtual void              beginRenderPass(vk::CommandBuffer cmd);
  virtual void              updateCameraUniform(int idx);
  virtual void              updateUniform(int idx);

  virtual void prepareFrame();
  virtual void presentFrame();

protected:
  std::string mName;
  uint32_t    mApiVersion;
  bool        mPrepared;
  bool        mEndApplication = true;

  std::unique_ptr<Instance>      mInstance;
  std::unique_ptr<Window>        mWindow;
  vk::SurfaceKHR                 mVkSurface;
  std::unique_ptr<Device>        mDevice;
  std::unique_ptr<RenderContext> mRenderContext;
  vk::RenderPass                 mVkRenderPass;
  std::vector<vk::Framebuffer>   mVkFramebuffers;
  vk::DescriptorPool             mVkDescPool;

  vk::Fence          mCurrCmdFence;
  BufferingSemaphore mCurrBufferingSemaphore;
  uint32_t           mCurrBufferingIdx;

  uint32_t                       mLastSubpass;
  std::unique_ptr<GUI>           mGUI;
  std::unique_ptr<InputCallback> mInputCallback;

  //used for camera
  std::unique_ptr<GraphicsCamera> mGraphicsCamera;
  struct CameraDescriptorSet {
    DescriptorSetLayout*           descLayout{nullptr};
    std::vector<vk::DescriptorSet> descSets{};
    std::unique_ptr<UniformBuffer> cameraUB{nullptr};
  } mCameraDescriptor;

public:
  Device*        getDevice() { return mDevice.get(); }
  RenderContext* getRenderContext() { return mRenderContext.get(); }
};
}  //namespace vkl
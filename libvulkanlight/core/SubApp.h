#pragma once
#include <functional>
#include <thread>
#include <forward_list>
#include <vulkan/vulkan.hpp>
#include "vkltypes.h"

namespace vkl {
class Device;
class Window;
class RenderContext;
class SubApp {
public:
  SubApp();
  virtual ~SubApp();

  void registerWindow(std::unique_ptr<Window>& _window);

  virtual bool prepare(Device* device, vk::DescriptorPool descPool);
  virtual void run();
  virtual void end();

protected:
  void         createRenderContext();
  virtual void createCommandPool();
  virtual void createCommandBuffer();
  virtual void createRenderPass();
  virtual void createFrameBuffer();
  virtual void createPipelines();
  virtual void createRenderers();
  virtual void buildCommandBuffer();

  virtual void updateUniforms(uint32_t idx);
  virtual void onRender();
  virtual void prepareFrame();
  virtual void submitCommandBuffer();

public:

protected:
  std::unique_ptr<Window>        mWindow;
  Device*                        mDevice;
  std::unique_ptr<RenderContext> mRenderContext;
  vk::RenderPass                 mVkRenderPass;
  std::vector<vk::Framebuffer>   mVkFramebuffers;
  vk::DescriptorPool             mVkDescPool;
  vk::Fence                      mCurrCmdFence;
  BufferingSemaphore             mCurrBufferingSemaphore;
  uint32_t                       mCurrBufferingIdx;
  std::thread                    mThread;
  bool                           mEndApplication;
  bool                           mPrepared;
};
}  //namespace vkl
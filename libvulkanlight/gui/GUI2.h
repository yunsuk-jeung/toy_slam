#pragma once

#include <vulkan/vulkan.hpp>
#include "Image.h"
#include "VkBaseRenderer.h"
namespace vkl {
class InputCallback;
class Device;
class RenderContext;
class Window;
class GraphicsCamera;
class Buffer;
class GUI2 : public VkBaseRenderer {
public:
  GUI2();
  ~GUI2();
  virtual void onWindowResized(int w, int h) override;

  virtual void initialize(Device*            device,
                          RenderContext*     context,
                          vk::DescriptorPool descPool) override;
  virtual void buildCommandBuffer(vk::CommandBuffer cmdBuffer, uint32_t idx) override;
  virtual void onRender() override;
  virtual void addInputCallback(InputCallback* cb);
  virtual void handleInputEvents();

protected:
  virtual void setName() override;
  virtual void setShader() override;
  virtual void createVkDescriptorSetLayout() override;
  virtual void createVkPipelineLayout() override;
  virtual void createVkPipeline(vk::RenderPass renderPass) override;
  virtual void createVkDescriptorSets() override;
  virtual void updateDescriptorSets() override;

  virtual void createVertexBuffers() override;
  virtual void createIndexBuffers() override;
  virtual void createUniformBuffers() override;
  virtual void createTextures() override;

  void updateImGuiBuffer(uint32_t idx);

  //void createInputCallback();

public:
  Window* window = nullptr;

  //handling input events
  std::vector<InputCallback*> inputCallbacks;

  //used for imgui
  std::vector<size_t>                  prevVBSizes;
  std::vector<size_t>                  prevIBSizes;
  std::vector<std::unique_ptr<Buffer>> VBs;
  std::vector<std::unique_ptr<Buffer>> IBs;
  std::unique_ptr<Image>               fontImage;
  vk::Sampler                          fontSampler      = VK_NULL_HANDLE;
  vk::DescriptorSet                    fontDescSet      = VK_NULL_HANDLE;
  vk::DescriptorSetLayout              texDescSetLayout = VK_NULL_HANDLE;
  vk::PushConstantRange                pushConstants;

  vk::CommandBuffer guiCmdBuffer = VK_NULL_HANDLE;
};
}  //namespace vkl
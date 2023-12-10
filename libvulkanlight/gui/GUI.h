#pragma once
#include "RendererBase.h"
#include "macros.h"
namespace ImGui {
class Object;
}
namespace vkl {
class InputCallback;
class Window;
class Buffer;
class GUI : public RendererBase {
public:
  USING_SMART_PTR(GUI);
  GUI();
  ~GUI();

  void onWindowResized(int w, int h) override;

  void prepare(Device*            device,
               RenderContext*     context,
               vk::DescriptorPool descPool,
               vk::RenderPass     vkRenderPass,
               Pipeline*          pipeline) override;

  virtual void addInputCallback(InputCallback* cb);
  void         addImGuiObjects(std::shared_ptr<ImGui::Object> objects);

  void onRender();

  void buildCommandBuffer(vk::CommandBuffer cmd, uint32_t currBufferingIdx);

protected:
  void updateBuffer(uint32_t idx);
  void handleInputCallbacks();

  void setName() override;
  void createVertexBuffer() override;
  void createIndexBuffers() override;
  void createTextures() override;
  void createDescriptorsets() override;
  void updateDescriptorsets() override;

protected:
  Window* mWindow;

  std::vector<InputCallback*> mInputCallbacks;

  std::vector<size_t>                  mPrevVBSizes;
  std::vector<size_t>                  mPrevIBSizes;
  std::vector<std::unique_ptr<Buffer>> mVBs;
  std::vector<std::unique_ptr<Buffer>> mIBs;
  std::unique_ptr<Image>               mFontImage;
  vk::Sampler                          mFontSampler;
  vk::DescriptorSet                    mFontDescSet;

  std::vector<std::shared_ptr<ImGui::Object>> mImGuiObjects;
};
}  //namespace vkl
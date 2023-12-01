#pragma once
#include "RendererBase.h"

namespace vkl {
class Window;
class Buffer;
class GUI : public RendererBase {
public:
  GUI();
  ~GUI();

  void onWindowResized(int w, int h) override;

  void prepare(Device*            device,
               RenderContext*     context,
               vk::DescriptorPool descPool,
               vk::RenderPass     vkRenderPass,
               std::string        pipelineName) override;

  void onRender();

protected:
  void setName() override;
  void createVertexBuffer() override;
  void createIndexBuffers() override;
  //void createTextures();
  //void createDescriptorsets();

protected:
  Window* mWindow;

  std::vector<size_t>                  mPrevVBSizes;
  std::vector<size_t>                  mPrevIBSizes;
  std::vector<std::unique_ptr<Buffer>> mVBs;
  std::vector<std::unique_ptr<Buffer>> mIBs;
};
}  //namespace vkl
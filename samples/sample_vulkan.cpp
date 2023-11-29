#include <iostream>
#include "Logger.h"
#include "App.h"
#include "VkShaderUtil.h"

namespace vkl {
class SampleApp : public App {
public:
  SampleApp() {}
  ~SampleApp() {}

  void onWindowResized(int w, int h, int orientation = 0) override {
    mName       = "Compute Application";
    mApiVersion = VK_API_VERSION_1_1;
    GLSLCompiler::set_target_environment(glslang::EShTargetSpv,
                                         glslang::EShTargetSpv_1_3);
  }
  bool prepare() override {}
  void run() override {}
  void onRender() override {}

  void buildCommandBuffer() override {}
  void updateUniform(int idx) override {}

protected:
};
}  //namespace vkl

int main() {
  auto hi = " hi hi hi";
  std::cout << hi << std::endl;
  return 0;
}
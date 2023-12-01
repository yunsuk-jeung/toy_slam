#pragma once
#include "RendererBase.h"

namespace vkl {
class BasicRenderer : public RendererBase {
public:
  BasicRenderer();
  virtual ~BasicRenderer();

protected:
  virtual void setName() override;
  virtual void setShader() override;
};
}  //namespace vkl
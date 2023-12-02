#pragma once
#include "VklLogger.h"
#include "Device.h"
#include "RenderContext.h"
#include "ShaderModule.h"
#include "PipelineLayout.h"
#include "ShaderTypes.h"
#include "BasicRenderer.h"
namespace vkl {
BasicRenderer::BasicRenderer()
  : RendererBase() {}

BasicRenderer::~BasicRenderer() {}

void BasicRenderer::setName() {
  mName = "Basic Renderer";
  VklLogI("Basic Renderer");
}
}  //namespace vkl
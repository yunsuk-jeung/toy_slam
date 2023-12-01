#pragma once
#include "VkLogger.h"
#include "BasicRenderer.h"
namespace vkl {
BasicRenderer::BasicRenderer()
  : RendererBase() {}

BasicRenderer::~BasicRenderer() {}

void BasicRenderer::setName() {
  mName = "Basic Renderer";
  VklLogI("Basic Renderer");
}

void BasicRenderer::setShader() {
  mShaderName       = "Basic";
  mShaderSrcType    = ShaderSourceType::STRING;
  mVertShaderSource = {
    "#version 450 \n"
    "#include \"ShaderTypes.h\" \n"
    "layout(location = 0) in vec3 i_pos;      \n"
    "layout(location = 1) in vec3 i_col;      \n"
    "layout(location = 2) in vec2 i_uv;      \n"

    "layout(set = 0, binding = 0, std140) uniform camData{ \n"
    "  CameraUniform cam;};\n "
    "layout(set = 1, binding = 0, std140) uniform modelMat{ \n"
    "  mat4 model;};\n "

    "layout(location = 0) out vec3 o_col;     \n"
    "layout(location = 1) out vec2 o_uv;      \n"
    "void main() {                            \n"
    "    gl_Position = cam.P * cam.V * cam. M * model * vec4(i_pos, 1.0); \n"
    "    o_col = i_col;                       \n"
    "    o_uv = i_uv;                         \n"
    "}                                        \n"};

  mFragShaderSource = {
    "#version 450                             \n"
    "precision mediump float;                      \n"
    "#include \"ShaderTypes.h\" \n"
    "layout(location = 0) in vec3 i_col;           \n"
    "layout(location = 1) in vec2 i_uv;                  \n"

    "layout(location = 0) out vec4 FragColor0;     \n"

    "layout(set = 2, binding = 0, std140) uniform materialData{\n"
    "  SampleMaterial material;};\n"
    "layout(set = 3, binding = 0) uniform sampler textureSampler;\n"
    "layout(set = 3, binding = 1) uniform texture2D textureImage;\n"

    "void main() {                                 \n"
    "  vec4 col = vec4(i_col,1.0);                             \n"
    "  col.xyz *= material.col.xyz;                            \n"
    "  col *= texture(sampler2D(textureImage, textureSampler), i_uv).rgba;\n"
    "  FragColor0 = col; \n"
    "}                                             \n"};
}
}  //namespace vkl
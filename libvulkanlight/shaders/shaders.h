#pragma once
#include <string>

namespace vkl {
namespace shader {
const std::string basicVert = R"(
#version 450
#include "ShaderTypes.h"
layout(location = 0) in vec3 i_pos;
layout(location = 1) in vec3 i_col;
layout(location = 2) in vec2 i_uv;

layout(set = 0, binding = 0, std140) uniform camData{
  CameraUniform cam;};
layout(set = 1, binding = 0, std140) uniform modelMat{
  mat4 model;};

layout(location = 0) out vec3 o_col;
layout(location = 1) out vec2 o_uv;
void main() {
    gl_Position = cam.P * cam.V * model * vec4(i_pos, 1.0);
    o_col = i_col;
    o_uv = i_uv;
}
)";
const std::string basicFrag = R"(
#version 450
precision mediump float;
#include "ShaderTypes.h"
layout(location = 0) in vec3 i_col;
layout(location = 1) in vec2 i_uv;

layout(location = 0) out vec4 FragColor0;

layout(set = 2, binding = 0, std140) uniform materialData{
  SampleMaterial material;};
layout(set = 3, binding = 0) uniform sampler textureSampler;
layout(set = 3, binding = 1) uniform texture2D textureImage;

void main() {
  vec4 col = vec4(i_col,1.0);
  col.xyz *= material.col.xyz;
  col *= texture(sampler2D(textureImage, textureSampler), i_uv).rgba;
  FragColor0 = col;
}
)";
}  //namespace shader
}  //namespace vkl
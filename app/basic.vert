#version 450
#include "ShaderTypes.h"
layout(location = 0) in vec3 i_pos;
layout(location = 1) in vec3 i_col;

layout(set = 0, binding = 0, std140) uniform camData{
  CameraUniform cam;};
layout(set = 1, binding = 0, std140) uniform modelMat{
  mat4 model;};

layout(location = 0) out vec3 o_col;
void main() {
    gl_Position = cam.P * cam.V * cam.M * model * vec4(i_pos, 1.0);
    o_col = i_col;
}
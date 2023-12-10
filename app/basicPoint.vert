#version 450
#include "ShaderTypes.h"

layout(location = 0) in vec4 i_pos;
layout(location = 0) out vec3 o_col;

layout(set = 0, binding = 0, std140) uniform camData{
  CameraUniform cam;};

layout(set = 1, binding = 0, std140) uniform modelMat{
  mat4 model;};

void main(){
    gl_Position  = cam.P * cam.V * model * vec4(i_pos.xyz,1.0);
    gl_PointSize = 3.0f;
    o_col = vec3(1.0, 1.0, 0.0);
}
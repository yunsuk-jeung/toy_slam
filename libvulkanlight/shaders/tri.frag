#version 450
precision mediump float;

layout(location = 0) in vec3 i_col;
layout(location = 0) out vec4 fragment_color0;

void main() {
  vec3 col        = i_col;
  fragment_color0 = vec4(col, 1.0);
}

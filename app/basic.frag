#version 450
precision mediump float;
layout(location = 0) in vec3 i_col;
layout(location = 0) out vec4 o_fragColor;
void main() {
  o_fragColor = vec4(i_col, 1.0);
}
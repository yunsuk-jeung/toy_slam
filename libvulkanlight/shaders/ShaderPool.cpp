#include <functional>
#include "utils.h"
#include "ShaderPool.h"
namespace vkl {

std::map<size_t, std::string> ShaderPool::shaderMap;

void ShaderPool::init() {
  std::hash<std::string> hasher;
  size_t                 key;
  std::string            name;
  std::string            shader;
  name   = "pos_col_mvp_m_vert";
  shader = R"(
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
)";
  key    = hasher(name);
  shaderMap.insert({key, shader});

  name   = "pos_col_mvp_m_point_vert";
  shader = R"(
#version 450
#include "ShaderTypes.h"

layout(location = 0) in vec4 i_pos;
layout(location = 0) out vec3 o_col;

layout(set = 0, binding = 0, std140) uniform camData{
  CameraUniform cam;};

layout(set = 1, binding = 0, std140) uniform modelMat{
  mat4 model;};

void main(){
    gl_Position  = cam.P * cam.V * model * vec4(i_pos.xyz, 1.0);
    gl_PointSize = 3.0f;
    o_col = vec3(1.0, 1.0, 0.0);
}
)";
  key    = hasher(name);
  shaderMap.insert({key, shader});

  name   = "pos_col_nuv_mvp_m_vert";
  shader = R"(
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
  key    = hasher(name);
  shaderMap.insert({key, shader});

  //// fragments /////
  //// fragments /////
  //// fragments /////
  name   = "col_frag";
  shader = R"(
#version 450
precision mediump float;
layout(location = 0) in vec3 i_col;
layout(location = 0) out vec4 FragColor0;
void main(){
    FragColor0 = vec4(i_col, 1.0);
}
)";
  key    = hasher(name);
  shaderMap.insert({key, shader});

  name   = "col_uv_mat_tex_rgba_frag";
  shader = R"(
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
  key    = hasher(name);
  shaderMap.insert({key, shader});

  /*
   * imgui
   */

  const uint32_t imguiVertspv[] =
    {0x07230203, 0x00010000, 0x00080001, 0x0000002e, 0x00000000, 0x00020011, 0x00000001,
     0x0006000b, 0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e,
     0x00000000, 0x00000001, 0x000a000f, 0x00000000, 0x00000004, 0x6e69616d, 0x00000000,
     0x0000000b, 0x0000000f, 0x00000015, 0x0000001b, 0x0000001c, 0x00030003, 0x00000002,
     0x000001c2, 0x00040005, 0x00000004, 0x6e69616d, 0x00000000, 0x00030005, 0x00000009,
     0x00000000, 0x00050006, 0x00000009, 0x00000000, 0x6f6c6f43, 0x00000072, 0x00040006,
     0x00000009, 0x00000001, 0x00005655, 0x00030005, 0x0000000b, 0x0074754f, 0x00040005,
     0x0000000f, 0x6c6f4361, 0x0000726f, 0x00030005, 0x00000015, 0x00565561, 0x00060005,
     0x00000019, 0x505f6c67, 0x65567265, 0x78657472, 0x00000000, 0x00060006, 0x00000019,
     0x00000000, 0x505f6c67, 0x7469736f, 0x006e6f69, 0x00030005, 0x0000001b, 0x00000000,
     0x00040005, 0x0000001c, 0x736f5061, 0x00000000, 0x00060005, 0x0000001e, 0x73755075,
     0x6e6f4368, 0x6e617473, 0x00000074, 0x00050006, 0x0000001e, 0x00000000, 0x61635375,
     0x0000656c, 0x00060006, 0x0000001e, 0x00000001, 0x61725475, 0x616c736e, 0x00006574,
     0x00030005, 0x00000020, 0x00006370, 0x00040047, 0x0000000b, 0x0000001e, 0x00000000,
     0x00040047, 0x0000000f, 0x0000001e, 0x00000002, 0x00040047, 0x00000015, 0x0000001e,
     0x00000001, 0x00050048, 0x00000019, 0x00000000, 0x0000000b, 0x00000000, 0x00030047,
     0x00000019, 0x00000002, 0x00040047, 0x0000001c, 0x0000001e, 0x00000000, 0x00050048,
     0x0000001e, 0x00000000, 0x00000023, 0x00000000, 0x00050048, 0x0000001e, 0x00000001,
     0x00000023, 0x00000008, 0x00030047, 0x0000001e, 0x00000002, 0x00020013, 0x00000002,
     0x00030021, 0x00000003, 0x00000002, 0x00030016, 0x00000006, 0x00000020, 0x00040017,
     0x00000007, 0x00000006, 0x00000004, 0x00040017, 0x00000008, 0x00000006, 0x00000002,
     0x0004001e, 0x00000009, 0x00000007, 0x00000008, 0x00040020, 0x0000000a, 0x00000003,
     0x00000009, 0x0004003b, 0x0000000a, 0x0000000b, 0x00000003, 0x00040015, 0x0000000c,
     0x00000020, 0x00000001, 0x0004002b, 0x0000000c, 0x0000000d, 0x00000000, 0x00040020,
     0x0000000e, 0x00000001, 0x00000007, 0x0004003b, 0x0000000e, 0x0000000f, 0x00000001,
     0x00040020, 0x00000011, 0x00000003, 0x00000007, 0x0004002b, 0x0000000c, 0x00000013,
     0x00000001, 0x00040020, 0x00000014, 0x00000001, 0x00000008, 0x0004003b, 0x00000014,
     0x00000015, 0x00000001, 0x00040020, 0x00000017, 0x00000003, 0x00000008, 0x0003001e,
     0x00000019, 0x00000007, 0x00040020, 0x0000001a, 0x00000003, 0x00000019, 0x0004003b,
     0x0000001a, 0x0000001b, 0x00000003, 0x0004003b, 0x00000014, 0x0000001c, 0x00000001,
     0x0004001e, 0x0000001e, 0x00000008, 0x00000008, 0x00040020, 0x0000001f, 0x00000009,
     0x0000001e, 0x0004003b, 0x0000001f, 0x00000020, 0x00000009, 0x00040020, 0x00000021,
     0x00000009, 0x00000008, 0x0004002b, 0x00000006, 0x00000028, 0x00000000, 0x0004002b,
     0x00000006, 0x00000029, 0x3f800000, 0x00050036, 0x00000002, 0x00000004, 0x00000000,
     0x00000003, 0x000200f8, 0x00000005, 0x0004003d, 0x00000007, 0x00000010, 0x0000000f,
     0x00050041, 0x00000011, 0x00000012, 0x0000000b, 0x0000000d, 0x0003003e, 0x00000012,
     0x00000010, 0x0004003d, 0x00000008, 0x00000016, 0x00000015, 0x00050041, 0x00000017,
     0x00000018, 0x0000000b, 0x00000013, 0x0003003e, 0x00000018, 0x00000016, 0x0004003d,
     0x00000008, 0x0000001d, 0x0000001c, 0x00050041, 0x00000021, 0x00000022, 0x00000020,
     0x0000000d, 0x0004003d, 0x00000008, 0x00000023, 0x00000022, 0x00050085, 0x00000008,
     0x00000024, 0x0000001d, 0x00000023, 0x00050041, 0x00000021, 0x00000025, 0x00000020,
     0x00000013, 0x0004003d, 0x00000008, 0x00000026, 0x00000025, 0x00050081, 0x00000008,
     0x00000027, 0x00000024, 0x00000026, 0x00050051, 0x00000006, 0x0000002a, 0x00000027,
     0x00000000, 0x00050051, 0x00000006, 0x0000002b, 0x00000027, 0x00000001, 0x00070050,
     0x00000007, 0x0000002c, 0x0000002a, 0x0000002b, 0x00000028, 0x00000029, 0x00050041,
     0x00000011, 0x0000002d, 0x0000001b, 0x0000000d, 0x0003003e, 0x0000002d, 0x0000002c,
     0x000100fd, 0x00010038};
  name                           = "imgui_vert";
  const std::string vertexShader = std::string((const char*)imguiVertspv,
                                               sizeof(imguiVertspv));

  key = hasher(name);
  shaderMap.insert({key, vertexShader});

  const uint32_t imguiFragspv[] =
    {0x07230203, 0x00010000, 0x00080001, 0x0000001e, 0x00000000, 0x00020011, 0x00000001,
     0x0006000b, 0x00000001, 0x4c534c47, 0x6474732e, 0x3035342e, 0x00000000, 0x0003000e,
     0x00000000, 0x00000001, 0x0007000f, 0x00000004, 0x00000004, 0x6e69616d, 0x00000000,
     0x00000009, 0x0000000d, 0x00030010, 0x00000004, 0x00000007, 0x00030003, 0x00000002,
     0x000001c2, 0x00040005, 0x00000004, 0x6e69616d, 0x00000000, 0x00040005, 0x00000009,
     0x6c6f4366, 0x0000726f, 0x00030005, 0x0000000b, 0x00000000, 0x00050006, 0x0000000b,
     0x00000000, 0x6f6c6f43, 0x00000072, 0x00040006, 0x0000000b, 0x00000001, 0x00005655,
     0x00030005, 0x0000000d, 0x00006e49, 0x00050005, 0x00000016, 0x78655473, 0x65727574,
     0x00000000, 0x00040047, 0x00000009, 0x0000001e, 0x00000000, 0x00040047, 0x0000000d,
     0x0000001e, 0x00000000, 0x00040047, 0x00000016, 0x00000022, 0x00000000, 0x00040047,
     0x00000016, 0x00000021, 0x00000000, 0x00020013, 0x00000002, 0x00030021, 0x00000003,
     0x00000002, 0x00030016, 0x00000006, 0x00000020, 0x00040017, 0x00000007, 0x00000006,
     0x00000004, 0x00040020, 0x00000008, 0x00000003, 0x00000007, 0x0004003b, 0x00000008,
     0x00000009, 0x00000003, 0x00040017, 0x0000000a, 0x00000006, 0x00000002, 0x0004001e,
     0x0000000b, 0x00000007, 0x0000000a, 0x00040020, 0x0000000c, 0x00000001, 0x0000000b,
     0x0004003b, 0x0000000c, 0x0000000d, 0x00000001, 0x00040015, 0x0000000e, 0x00000020,
     0x00000001, 0x0004002b, 0x0000000e, 0x0000000f, 0x00000000, 0x00040020, 0x00000010,
     0x00000001, 0x00000007, 0x00090019, 0x00000013, 0x00000006, 0x00000001, 0x00000000,
     0x00000000, 0x00000000, 0x00000001, 0x00000000, 0x0003001b, 0x00000014, 0x00000013,
     0x00040020, 0x00000015, 0x00000000, 0x00000014, 0x0004003b, 0x00000015, 0x00000016,
     0x00000000, 0x0004002b, 0x0000000e, 0x00000018, 0x00000001, 0x00040020, 0x00000019,
     0x00000001, 0x0000000a, 0x00050036, 0x00000002, 0x00000004, 0x00000000, 0x00000003,
     0x000200f8, 0x00000005, 0x00050041, 0x00000010, 0x00000011, 0x0000000d, 0x0000000f,
     0x0004003d, 0x00000007, 0x00000012, 0x00000011, 0x0004003d, 0x00000014, 0x00000017,
     0x00000016, 0x00050041, 0x00000019, 0x0000001a, 0x0000000d, 0x00000018, 0x0004003d,
     0x0000000a, 0x0000001b, 0x0000001a, 0x00050057, 0x00000007, 0x0000001c, 0x00000017,
     0x0000001b, 0x00050085, 0x00000007, 0x0000001d, 0x00000012, 0x0000001c, 0x0003003e,
     0x00000009, 0x0000001d, 0x000100fd, 0x00010038};
  name   = "imgui_frag";
  shader = std::string((const char*)imguiFragspv, sizeof(imguiFragspv));

  key = hasher(name);
  shaderMap.insert({key, shader});
}
const std::string& ShaderPool::requestShaderStr(const std::string& name) {
  std::hash<std::string> hasher;
  size_t                 key = hasher(name);

  if (shaderMap.count(key) < 1) {
    VKL_ASSERT_MESSAGE(0, "non existing shader");
  }

  return shaderMap[key];
}
//glsl_shader.frag, compiled with:
//# glslangValidator -V -x -o glsl_shader.frag.u32 glsl_shader.frag
/*
#version 450 core
layout(location = 0) out vec4 fColor;
layout(set=0, binding=0) uniform sampler2D sTexture;
layout(location = 0) in struct { vec4 Color; vec2 UV; } In;
void main()
{
    fColor = In.Color * texture(sTexture, In.UV.st);
}
*/
//glsl_shader.vert, compiled with:
//# glslangValidator -V -x -o glsl_shader.vert.u32 glsl_shader.vert
/*
#version 450 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec4 aColor;
layout(push_constant) uniform uPushConstant { vec2 uScale; vec2 uTranslate; }
pc;

out gl_PerVertex { vec4 gl_Position; };
layout(location = 0) out struct { vec4 Color; vec2 UV; } Out;

void main()
{
    Out.Color = aColor;
    Out.UV = aUV;
    gl_Position = vec4(aPos * pc.uScale + pc.uTranslate, 0, 1);
}
*/
}  //namespace vkl
#ifndef SHADER_TYPES_H
#define SHADER_TYPES_H

#ifdef __cplusplus
#include <Eigen/Dense>
extern "C" {
#endif

#ifdef __cplusplus
struct VertexColorNuv {
  float x, y, z;
  float r, g, b;
  float nu, nv;
};
#endif

#ifdef __cplusplus
struct VertexColor {
  float x, y, z;
  float r, g, b;
};

struct Vertex4d {
  float x, y, z, w;
  
};
#endif

#ifdef __cplusplus
struct CameraUniform {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4f M;
  Eigen::Matrix4f V;
  Eigen::Matrix4f P;
};
#else
struct CameraUniform {
  mat4 M;
  mat4 V;
  mat4 P;
};
#endif
#ifdef __cplusplus
struct SampleMaterial {
  float r{1.0f}, g{1.0f}, b{1.0f};
  SampleMaterial(float _r = 1.0f, float _g = 1.0f, float _b = 1.0f)
    : r(_r)
    , g(_g)
    , b(_b) {}
};
#else
struct SampleMaterial {
  vec3 col;
};
#endif

#ifdef __cplusplus
}
#endif

#endif  //SHADER_TYPES_H
#pragma once
#include <Eigen/Dense>

namespace Eigen {

template <typename T>
static Eigen::Matrix<T, 3, 3> skew(Eigen::Matrix<T, 3, 1>& src) {
  Eigen::Matrix<T, 3, 3> skew = Eigen::Matrix3d::Zero();
  skew(0, 1)                  = -src[2];
  skew(0, 2)                  = src[1];
  skew(1, 2)                  = -src[0];
  skew(1, 0)                  = src[2];
  skew(2, 0)                  = -src[1];
  skew(2, 1)                  = src[0];
  return skew;
}
}  //namespace Eigen
#pragma once
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include "config.h"

namespace toy {
class BasicSolver {
public:
  static bool triangulate(const Eigen::Vector3d undist0,
                          const Eigen::Vector3d undist1,
                          const Sophus::SE3d&   Sc1c0,
                          Eigen::Vector3d&      out) {
    Eigen::Matrix<double, 3, 4> P1 = Eigen::Matrix<double, 3, 4>::Identity();
    Eigen::Matrix<double, 3, 4> P2 = Sc1c0.matrix3x4();
    Eigen::Matrix4d             A;
    A.row(0) = undist0[0] * P1.row(2) - undist0[2] * P1.row(0);
    A.row(1) = undist0[1] * P1.row(2) - undist0[2] * P1.row(1);
    A.row(2) = undist1[0] * P2.row(2) - undist1[2] * P2.row(0);
    A.row(3) = undist1[1] * P2.row(2) - undist1[2] * P2.row(1);

    Eigen::Vector4d homoPoint;
    homoPoint = A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    out       = homoPoint.head(3) / homoPoint.w();

    auto& min = Config::Solver::basicMinDepth;
    auto& max = Config::Solver::basicMaxDepth;

    if (out.z() < min || out.z() > max)
      return false;

    return true;
  }
};
}  //namespace toy

#pragma once
#include <Eigen/Dense>
namespace Eigen {
using Matrix3Pf = Eigen::Matrix<float, 3, 52>;
using MatrixP3f = Eigen::Matrix<float, 52, 3>;
using Matrix2Pf = Eigen::Matrix<float, 2, 52>;
using VectorPf  = Eigen::Matrix<float, 52, 1>;
using Matrix66d = Eigen::Matrix<double, 6, 6>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;
using Matrix62d = Eigen::Matrix<double, 6, 2>;
using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector6d  = Eigen::Matrix<double, 6, 1>;
}  //namespace Eigen
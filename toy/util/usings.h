#pragma once
#include <Eigen/Dense>
namespace Eigen {
using Matrix66d  = Matrix<double, 6, 6>;
using Matrix26d = Matrix<double, 2, 6>;
using Matrix23d = Matrix<double, 2, 3>;
using Matrix36d = Matrix<double, 3, 6>;

using Vector6d = Matrix<double, 6, 1>;
}  //namespace Eigen
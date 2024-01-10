#include <memory>
#include <iostream>
#include <string>
#include <iomanip>
#include "DebugUtil.h"

auto main() -> int {
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd I2(100, 100);
  I2.setIdentity();

  toy::debug::drawSparseMatrix("test", I2);
  return 0;
}
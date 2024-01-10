#include <memory>
#include <iostream>
#include <string>
#include <iomanip>
#include "DebugUtil.h"

auto main() -> int {
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  toy::debug::darwSparseMatrix("test", I);

  return 0;
}
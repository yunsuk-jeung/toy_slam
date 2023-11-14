#pragma once
#include <memory>

namespace toy {
class VioSolver;
class VioSolverFactory {
public:
  static std::unique_ptr<VioSolver> createVioSolver();
};

}  //namespace toy
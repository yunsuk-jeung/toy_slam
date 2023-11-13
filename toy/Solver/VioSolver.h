#pragma once
#include <memory>

namespace toy {
class VioSolver {
public:
  VioSolver() {}
  virtual ~VioSolver() {}

  virtual bool process() = 0;
};

class VioSolverFactory {
public:
  enum SolverType { SQRT, CERES };
  static std::unique_ptr<VioSolver> createVioSolver(bool useDouble, SolverType type);
};

}  //namespace toy
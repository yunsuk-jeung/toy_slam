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
  static VioSolver* createVioSolver();
};

}  //namespace toy
#pragma once

namespace toy {
class VioSolver;
class VioSolverFactory {
public:
  static VioSolver* createVioSolver();
};

}  //namespace toy
#pragma once
#include <memory>

namespace toy {
class LocalSolver {
public:
  LocalSolver() {}
  virtual ~LocalSolver() {}

  virtual bool process() = 0;
};

class LocalSolverFactory {
public:
  enum SolverType { SQRT, CERES };
  static std::unique_ptr<LocalSolver> createLocalSolver(bool useDouble, SolverType type);
  //static LocalSolver* createLocalSolver(bool useDouble, SolverType type);
};

}  //namespace toy
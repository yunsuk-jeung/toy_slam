#include <iostream>
#include "ToyLogger.h"
#include "config.h"
#include "SqrtLocalSolver.h"
#include "VioSolver.h"

namespace toy {
VioSolver::Uni VioSolverFactory::createVioSolver() {
  if (Config::Vio::solverType == "SqrtLocalSolver") {
    return std::make_unique<SqrtLocalSolver>();
  }

  ToyLogE("Check your json for vio solver");
  throw std::runtime_error("no vio solver type");
}
}  //namespace toy
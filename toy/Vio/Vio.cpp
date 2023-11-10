#include "Vio.h"
#include "LocalSolver.h"
#include "config.h"

namespace toy {
Vio::Vio() {

  auto solverType = static_cast<LocalSolverFactory::SolverType>(Config::localSolverType);
  optimzerUPtr    = LocalSolverFactory::createLocalSolver(Config::useDouble, solverType);
  optimzerUPtr->process();
}

Vio::~Vio(){};
}  //namespace toy
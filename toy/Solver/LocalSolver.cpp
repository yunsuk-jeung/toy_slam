#include <iostream>
#include "LocalSolverFactory.h"
#include "SqrtWindowSolver.h"

namespace toy {
std::unique_ptr<LocalSolver> LocalSolverFactory::createLocalSolver(bool       useDouble,
                                                                   SolverType type) {

  std::unique_ptr<LocalSolver> solver = nullptr;

  switch (type) {
  case toy::LocalSolverFactory::SolverType::SQRT:
    if (useDouble) {
    }
    else {
      solver = std::make_unique<SqrtWindowSolver<float>>();
    }
    break;
  case toy::LocalSolverFactory::SolverType::CERES:
    //a = new SqrtWindowSolver<float>();

    break;
  default:
    break;
  }
  solver = std::make_unique<SqrtWindowSolver<float>>();

  return solver;
}

//LocalSolver* LocalSolverFactory::createLocalSolver(bool useDouble, SolverType type) {
//  return nullptr;
//}

}  //namespace toy
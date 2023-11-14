#include <iostream>
#include "SqrtWindowSolver.h"

namespace toy {
std::unique_ptr<VioSolver> VioSolverFactory::createVioSolver(bool       useDouble,
                                                             SolverType type) {

  std::unique_ptr<VioSolver> solver = nullptr;

  switch (type) {
  case toy::VioSolverFactory::SolverType::SQRT:
    if (useDouble) {
    }
    else {
      solver = std::make_unique<SqrtWindowSolver<float>>();
    }
    break;
  case toy::VioSolverFactory::SolverType::CERES:
    //a = new SqrtWindowSolver<float>();

    break;
  default:
    break;
  }
  solver = std::make_unique<SqrtWindowSolver<float>>();

  return solver;
}

//VioSolver* VioSolverFactory::createVioSolver(bool useDouble, SolverType type)
//{
//  return nullptr;
//}

}  //namespace toy
#pragma once
#include "VioSolver.h"
namespace toy {
template <typename FLOAT>
class SqrtWindowSolver : public VioSolver {
public:
  SqrtWindowSolver() {}
  virtual ~SqrtWindowSolver() {}

  virtual bool process() {
    if (std::is_same<FLOAT, float>::value) {
      std::cout << "this is float sqrt\n";
    }
    else {
      std::cout << "this is double sqrt\n";
    }
    return true;
  }

protected:
};
}  //namespace toy

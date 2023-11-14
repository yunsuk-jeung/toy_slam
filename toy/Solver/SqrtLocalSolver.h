#pragma once
#include "VioSolver.h"
namespace toy {
class SqrtLocalSolver : public VioSolver {
public:
  SqrtLocalSolver() {}
  virtual ~SqrtLocalSolver() {}

  virtual bool process() {
    std::cout << "##########" << std::endl;
    return true;
  }

protected:
};
}  //namespace toy

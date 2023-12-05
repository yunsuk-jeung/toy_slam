#pragma once
#include "VioSolver.h"
namespace toy {
class SqrtLocalSolver : public VioSolver {
public:
  SqrtLocalSolver() {}
  virtual ~SqrtLocalSolver() {}

  virtual bool process() { return true; }

protected:
};
}  //namespace toy

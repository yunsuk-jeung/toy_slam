#pragma once
#include <memory>

namespace toy {
class VioSolver {
public:
  VioSolver() {}
  virtual ~VioSolver() {}

  virtual bool process() = 0;
};

}  //namespace toy
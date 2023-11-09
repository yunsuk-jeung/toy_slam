#pragma once

#include "Frame.h"
#include "Processor.h"

namespace toy {
class LocalSolver;
class Vio {
public:

  Vio();
  ~Vio();

private:

  std::unique_ptr<LocalSolver> optimzerUPtr;

};
}  //namespace toy
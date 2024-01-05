#pragma once
#include <memory>
#include <vector>
#include <map>
#include "macros.h"
namespace toy {
namespace db {
class Frame;
class MapPoint;
}  //namespace db
class VioSolver {
public:
  USING_SMART_PTR(VioSolver);
  VioSolver() {}
  virtual ~VioSolver() {}

  virtual bool solve(std::vector<std::shared_ptr<db::Frame>>&    frames,
                     std::vector<std::shared_ptr<db::MapPoint>>& mapPoints) = 0;

  virtual void marginalize(std::shared_ptr<db::Frame> frame) = 0;

protected:
};

class VioSolverFactory {
public:
  static VioSolver::Uni createVioSolver();
};

}  //namespace toy
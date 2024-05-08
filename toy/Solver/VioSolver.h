#pragma once
#include <memory>
#include <vector>
#include <set>
#include <forward_list>
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

  virtual bool solve(const std::vector<std::shared_ptr<db::Frame>>&    frames,
                     const std::vector<std::shared_ptr<db::MapPoint>>& mapPoints) = 0;

  virtual void marginalize(
    std::set<int64_t>&                                marginalkeyFrameIds,
    std::forward_list<std::shared_ptr<db::MapPoint>>& marginalMapPoints) = 0;

protected:
};

class VioSolverFactory {
public:
  static VioSolver::Uni createVioSolver();
};

}  //namespace toy
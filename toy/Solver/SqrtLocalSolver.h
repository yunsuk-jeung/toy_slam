#pragma once
#include "VioSolver.h"
namespace toy {
class SqrtLocalSolver : public VioSolver {
public:
  SqrtLocalSolver();
  virtual ~SqrtLocalSolver();

  virtual bool solve(std::vector<std::shared_ptr<db::Frame>>&    frames,
                     std::vector<std::shared_ptr<db::MapPoint>>& mapPoints);

protected:
  std::map<int, std::pair<int, int>> mFrameRows;
  std::map<int, std::pair<int, int>> mMapPointRows;
};
}  //namespace toy

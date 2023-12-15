#pragma once
#include "sophus/se3.hpp"
#include "usings.h"
#include "Parameter.h"
#include "VioSolver.h"

namespace toy {
class SqrtLocalSolver : public VioSolver {
public:
  SqrtLocalSolver();
  virtual ~SqrtLocalSolver();

  virtual bool solve(std::vector<std::shared_ptr<db::Frame>>&    frames,
                     std::vector<std::shared_ptr<db::MapPoint>>& mapPoints);

  virtual void marginalize(int id) override;

protected:
  void createStates(std::vector<std::shared_ptr<db::Frame>>&    frames,
                    std::vector<std::shared_ptr<db::MapPoint>>& mapPoints);

protected:
  std::map<int, FrameParameter>      mFrameParameterMap;
  std::map<int, std::pair<int, int>> mFrameRows;

  std::map<int, MapPointParameter>   mMapPointParameterMap;
  std::map<int, std::pair<int, int>> mMapPointRows;
};
}  //namespace toy

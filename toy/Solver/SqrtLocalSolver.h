#pragma once
#include "sophus/se3.hpp"
#include "usings.h"
#include "Parameter.h"
#include "VioSolver.h"

namespace toy {
class SqrtProblem;
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
  std::unique_ptr<SqrtProblem>     mProblem;
  std::map<int, FrameParameter>    mFrameParameterMap;
  std::map<int, MapPointParameter> mMapPointParameterMap;
};
}  //namespace toy

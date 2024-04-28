#pragma once
#include "sophus/se3.hpp"
#include "CustomTypes.h"
#include "VioSolver.h"

namespace toy {
class SqrtMarginalizer;
class SqrtMarginalizationCost;
class SqrtProblem;
class SqrtLocalSolver : public VioSolver {
public:
  SqrtLocalSolver();
  virtual ~SqrtLocalSolver();

  virtual bool solve(
    const std::vector<std::shared_ptr<db::Frame>>&    frames,
    const std::vector<std::shared_ptr<db::MapPoint>>& trackingMapPoints) override;

  virtual void marginalize(
    std::vector<std::shared_ptr<db::Frame>>&          marginalkeyFrames,
    std::forward_list<std::shared_ptr<db::MapPoint>>& lostMapPoints) override;

protected:

protected:
  std::unique_ptr<SqrtProblem>      mProblem;
  std::unique_ptr<SqrtMarginalizer> mMarginalizer;

  const std::vector<std::shared_ptr<db::Frame>>*    mFrames;
  const std::vector<std::shared_ptr<db::MapPoint>>* mMapPoints;

  //std::map<int, FrameParameter>    mFrameParameterMap;
  //std::map<int, MapPointParameter> mMapPointParameterMap;
};
}  //namespace toy

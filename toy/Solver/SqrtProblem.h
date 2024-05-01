#pragma once
#include <vector>
#include <map>
#include <Eigen/Dense>

#include "macros.h"

namespace toy {
namespace db {
class Frame;
class MapPoint;
}  //namespace db
class SqrtMarginalizationCost;
class PoseOnlyReporjectinCost;
class ReprojectionCost;
class MapPointLinearization;
class SqrtProblem {
public:
  USING_SMART_PTR(SqrtProblem);
  SqrtProblem();
  ~SqrtProblem();

  void reset();
  void setFrames(const std::vector<std::shared_ptr<db::Frame>>* framesRp);
  void setMapPoints(const std::vector<std::shared_ptr<db::MapPoint>>* mapPointsRp);

  void addPoseOnlyReprojectionCost(
    std::vector<std::shared_ptr<PoseOnlyReporjectinCost>>& costs);

  void addReprojectionCost(std::shared_ptr<db::MapPoint>                   mp,
                           std::vector<std::shared_ptr<ReprojectionCost>>& costs);

  void addMarginalizationCost(std::shared_ptr<SqrtMarginalizationCost> cost);

  std::vector<std::shared_ptr<MapPointLinearization>> grepMarginMapPointLinearizations(
    std::vector<std::shared_ptr<db::MapPoint>>& mps);

  bool solve();

  double linearize(bool updateState);
  void   decomposeLinearization();

protected:
  void constructFrameHessian();
  void backupParameters();
  void restoreParameters();

public:
  struct Option {
    Option();
    int    mMaxIteration;
    double mLambda;
    double mMaxLambda;
    double mMinLambda;
    double mMu;
    double mMuFactor;
  } mOption;

protected:
  //std::map<int, FrameParameter>*                      mFrameParameterMapRpt;
  //std::map<int, MapPointParameter>*                   mMapPointParameterMapRpt;
  std::map<int64_t, size_t>                         mFrameIdColumnMap;
  const std::vector<std::shared_ptr<db::Frame>>*    mFrames;
  const std::vector<std::shared_ptr<db::MapPoint>>* mMapPoints;

  std::vector<std::shared_ptr<MapPointLinearization>> mMapPointLinearizations;

  std::vector<std::shared_ptr<PoseOnlyReporjectinCost>> mPoseOnlyReprojectionCosts;
  std::shared_ptr<SqrtMarginalizationCost>              mSqrtMarginalizationCost;

  Eigen::MatrixXd mH;
  Eigen::VectorXd mB;

public:
  std::shared_ptr<SqrtMarginalizationCost> getSqrtMarginalizationCost() {
    return mSqrtMarginalizationCost;
  }

  std::map<int64_t, size_t>& getFrameIdColumnMap() { return mFrameIdColumnMap; };

  auto& mapPointLinearization() { return mMapPointLinearizations; }
};
}  //namespace toy
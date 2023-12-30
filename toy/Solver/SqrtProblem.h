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
class ReprojectionCost;
class MapPointLinearization;
class SqrtProblem {
public:
  USING_SMART_PTR(SqrtProblem);
  SqrtProblem();
  ~SqrtProblem();

  void reset();
  void setFrames(std::vector<std::shared_ptr<db::Frame>>* framesRp);
  void setMapPoints(std::vector<std::shared_ptr<db::MapPoint>>* mapPointsRp);

  void addReprojectionCost(std::shared_ptr<db::MapPoint>                  mp,
                           std::vector<std::shared_ptr<ReprojectionCost>> costs);

  bool solve();

protected:
  double linearize(bool updateState);
  void   decomposeLinearization();
  void   backupParameters();
  void   restoreParameters();

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
  std::map<int, int>                          mFrameIdColumnMap;
  std::vector<std::shared_ptr<db::Frame>>*    mFrames;
  std::vector<std::shared_ptr<db::MapPoint>>* mMapPoints;

  std::vector<std::shared_ptr<MapPointLinearization>> mMapPointLinearizations;

  Eigen::MatrixXd mH;
  Eigen::VectorXd mB;

public:
};
}  //namespace toy
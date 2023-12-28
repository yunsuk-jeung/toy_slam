#pragma once
#include <vector>
#include <map>
#include "macros.h"
#include "Parameter.h"

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

  void addReprojectionCost(std::shared_ptr<db::MapPoint>                  mp,
                           std::vector<std::shared_ptr<ReprojectionCost>> costs);

  bool solve();

protected:
  double linearize(bool updateState);
  void   decomposeLinearization();
  void   backupParameters();

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
  std::map<int, FrameParameter>*                        mRpFrameParameterMap;
  std::map<int, MapPointParameter>*                     mRpMapPointParameterMap;
  std::map<int, std::shared_ptr<MapPointLinearization>> mMapPointLinearizationMap;

  Eigen::MatrixXd mH;
  Eigen::VectorXd mB;

public:
  void setFrameSatatesMap(std::map<int, FrameParameter>* map) {
    mRpFrameParameterMap = map;
  }
  void setMapPointState(std::map<int, MapPointParameter>* map) {
    mRpMapPointParameterMap = map;
  }
};
}  //namespace toy
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
  SqrtProblem()  = default;
  ~SqrtProblem() = default;

  void addReprojectionCost(std::shared_ptr<db::MapPoint>                  mp,
                           std::vector<std::shared_ptr<ReprojectionCost>> costs);

  bool solve();

protected:
  double linearize(bool updateState);
  void   decomposeLinearization();

public:
  struct Option {
    Option();
    int    mMaxIteration;
    double mLambda;
    double mMaxLambda;
    double mMinLambda;
    double mMu;
  } mOption;

protected:
  std::map<int, FrameParameter>*                        mFrameParameterMap;
  std::map<int, MapPointParameter>*                     mMapPointParameterMap;
  std::map<int, std::shared_ptr<MapPointLinearization>> mMapPointLinearizationMap;

public:
  void setFrameSatatesMap(std::map<int, FrameParameter>* map) {
    mFrameParameterMap = map;
  }
  void setMapPointState(std::map<int, MapPointParameter>* map) {
    mMapPointParameterMap = map;
  }
};
}  //namespace toy
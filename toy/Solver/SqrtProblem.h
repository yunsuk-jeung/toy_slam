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

  void reset();
  void setFrameSatatesMap(std::map<int, FrameParameter>* map);
  void setMapPointState(std::map<int, MapPointParameter>* map);

  void addReprojectionCost(MapPointParameter*                             rpMpP,
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
  std::map<int, int>              mFrameIdColumnMap;
  std::vector<FrameParameter*>    mFrameParameters;
  std::vector<MapPointParameter*> mMapPointParameters;

  std::vector<std::shared_ptr<MapPointLinearization>> mMapPointLinearizations;

  Eigen::MatrixXd mH;
  Eigen::VectorXd mB;

public:
};
}  //namespace toy
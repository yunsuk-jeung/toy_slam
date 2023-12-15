#pragma once
#include <map>
#include "macros.h"
#include "Parameter.h"

namespace toy {
class SqrtProblem {
public:
  USING_SMART_PTR(SqrtProblem);
  SqrtProblem()  = default;
  ~SqrtProblem() = default;

protected:

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
  std::map<int, FrameParameter>*    mFrameParameterMap;
  std::map<int, MapPointParameter>* mMapPointParameterMap;

public:
  void setFrameSatatesMap(std::map<int, FrameParameter>* map) {
    mFrameParameterMap = map;
  }
  void setMapPointState(std::map<int, MapPointParameter>* map) {
    mMapPointParameterMap = map;
  }
};
}  //namespace toy
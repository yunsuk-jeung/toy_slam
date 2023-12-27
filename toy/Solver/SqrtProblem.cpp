#include "ToyLogger.h"
#include "SqrtProblem.h"
#include "MapPoint.h"
#include "CostFunction.h"
#include "MapPointLinearization.h"

namespace toy {
SqrtProblem::Option::Option()
  : mMaxIteration{7}
  , mLambda{1e-4}
  , mMaxLambda{1e2}
  , mMinLambda{1e-5}
  , mMu{2.0} {}

void SqrtProblem::addReprojectionCost(db::MapPoint::Ptr                  mp,
                                      std::vector<ReprojectionCost::Ptr> costs) {
  const auto& id                = mp->id();
  mMapPointLinearizationMap[id] = std::make_shared<MapPointLinearization>(costs);
}

bool SqrtProblem::solve() {
  double err = linearize(true);

  ToyLogD("initial err : {}", err);

  for (auto& [key, val] : *mFrameParameterMap) {
    val.backup();
  }

  for (auto& [key, val] : *mMapPointParameterMap) {
    val.backup();
  }

  decomposeLinearization();

  return true;
}

double SqrtProblem::linearize(bool updateState) {
  double errSq = 0;
  for (auto& [key, val] : mMapPointLinearizationMap) {
    //errSq += val->linearize(updateState);
    double err = val->linearize(updateState);

    ToyLogD("initial err : {}", err);
  }
  return errSq;
}

void SqrtProblem::decomposeLinearization() {
    for (auto& [key, val] : mMapPointLinearizationMap) {
    val->decomposeWithQR();
  }
}

}  //namespace toy

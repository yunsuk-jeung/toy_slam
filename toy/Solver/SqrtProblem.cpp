#include "ToyAssert.h"
#include "DebugUtil.h"
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
  , mMu{2.0}
  , mMuFactor{2.0} {}

SqrtProblem::SqrtProblem()
  : mRpFrameParameterMap{nullptr}
  , mRpMapPointParameterMap{nullptr} {}

SqrtProblem::~SqrtProblem() {}

void SqrtProblem::addReprojectionCost(db::MapPoint::Ptr                  mp,
                                      std::vector<ReprojectionCost::Ptr> costs) {
  TOY_ASSERT(mRpFrameParameterMap != nullptr);

  const auto& id = mp->id();

  mMapPointLinearizationMap[id] = std::make_shared<
    MapPointLinearization>(mRpFrameParameterMap, costs);
}

bool SqrtProblem::solve() {
  const int Hrows = mRpFrameParameterMap->size() * FrameParameter::SIZE;
  mH.resize(Hrows, Hrows);
  mB.resize(Hrows);

  double err = linearize(true);

  ToyLogD("initial err : {}", err);

  decomposeLinearization();

  for (auto& [key, val] : mMapPointLinearizationMap) {
    const Eigen::MatrixXd& vJ = val->J();
    const Eigen::VectorXd& vC = val->C();

    const auto  rows = vJ.rows() - MapPointParameter::SIZE;
    const auto& cols = Hrows;

    const Eigen::MatrixXd J = vJ.bottomLeftCorner(rows, cols);
    const Eigen::VectorXd C = vC.bottomRows(rows);

    Eigen::MatrixXd Jt = J.transpose();

    mH += Jt * J;
    mB -= Jt * C;
  }

  bool            deltaValid = false;
  Eigen::VectorXd frameDelta;

  auto& lambda    = mOption.mLambda;
  auto& minLambda = mOption.mMinLambda;
  auto& maxLambda = mOption.mMaxLambda;
  auto& mu        = mOption.mMu;
  auto& muFactor  = mOption.mMuFactor;

  for (int i = 0; i < 3 && !deltaValid; ++i) {
    Eigen::VectorXd lambdaVec = (mH.diagonal() * mOption.mLambda)
                                  .cwiseMax(mOption.mMinLambda);
    Eigen::MatrixXd H = mH;
    H.diagonal() += lambdaVec;

    frameDelta = H.ldlt().solve(mB);

    if (!frameDelta.array().isFinite().all()) {
      lambda = mu * lambda;
      mu *= muFactor;
    }
    else {
      deltaValid = true;
    }
  }

  if (!deltaValid) {
    ToyLogE("delta is unstable");
  }

  backupParameters();

  return true;
}

double SqrtProblem::linearize(bool updateState) {
  double errSq = 0;
  //YSTODO: tbb
  for (auto& [key, val] : mMapPointLinearizationMap) {
    //errSq += val->linearize(updateState);
    double err = val->linearize(updateState);
  }
  return errSq;
}

void SqrtProblem::decomposeLinearization() {
  //YSTODO: tbb
  for (auto& [key, val] : mMapPointLinearizationMap) {
    val->decomposeWithQR();
  }
#ifdef DEBUG_MATRIX0
  static int qridx = 0;
  int        temp  = 0;
  for (auto& [key, val] : mMapPointLinearizationMap) {
    debug::saveMatrix("qr" + std::to_string(qridx), temp++, val->J());
  }
  qridx++;
#endif
}
void SqrtProblem::backupParameters() {
  for (auto& [key, val] : *mRpFrameParameterMap) {
    val.backup();
  }

  for (auto& [key, val] : *mRpMapPointParameterMap) {
    val.backup();
  }
}
}  //namespace toy

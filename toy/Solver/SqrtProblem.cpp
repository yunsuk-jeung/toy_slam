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
  : mFrameParameters{}
  , mMapPointParameters{} {}

SqrtProblem::~SqrtProblem() {}

void SqrtProblem::reset() {
  mH.setZero();
  mB.setZero();

  mFrameIdColumnMap.clear();
  mFrameParameters.clear();
  mMapPointParameters.clear();
  mMapPointLinearizations.clear();
}

void SqrtProblem::setFrameSatatesMap(std::map<int, FrameParameter>* map) {
  mFrameParameters.reserve(map->size());
  int column = 0;
  for (auto& [key, val] : *map) {
    mFrameParameters.push_back(&val);
    mFrameIdColumnMap[val.id()] = column;
    column += FrameParameter::SIZE;
  }
}
void SqrtProblem::setMapPointState(std::map<int, MapPointParameter>* map) {
  mMapPointParameters.reserve(map->size());
  for (auto& [key, val] : *map) {
    mMapPointParameters.push_back(&val);
  }
}

void SqrtProblem::addReprojectionCost(MapPointParameter*                 rpMpP,
                                      std::vector<ReprojectionCost::Ptr> costs) {
  mMapPointLinearizations.emplace_back(
    std::make_shared<MapPointLinearization>(rpMpP, &mFrameIdColumnMap, costs));
}

bool SqrtProblem::solve() {
  const int Hrows = mFrameParameters.size() * FrameParameter::SIZE;
  mH.resize(Hrows, Hrows);
  mB.resize(Hrows);
  mH.setZero();
  mB.setZero();

  double err = linearize(true);

  ToyLogD("initial err : {}", err);

  decomposeLinearization();

  for (auto& mpL : mMapPointLinearizations) {
    const Eigen::MatrixXd& vJ = mpL->J();
    const Eigen::VectorXd& vC = mpL->C();

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
    Eigen::VectorXd lambdaVec = (mH.diagonal() * lambda).cwiseMax(minLambda);
    Eigen::MatrixXd H         = mH;
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

  double linearizedDiff = 0.0;
  for (auto& mpL : mMapPointLinearizations) {
    linearizedDiff += mpL->backSubstitue(frameDelta);
  }

  int frameRow = 0;
  for (auto& fp : mFrameParameters) {
    fp->update(frameDelta.segment<FrameParameter::SIZE>(frameRow));
    frameRow += FrameParameter::SIZE;
  }

  double newErr = linearize(false);

  ToyLogD("frame delta : {}", ToyLogger::eigenVec(frameDelta));
  ToyLogD("check err : {}  --> {}", err, newErr);

  return true;
}

double SqrtProblem::linearize(bool updateState) {
  double errSq = 0;
  //YSTODO: tbb
  for (auto& mpL : mMapPointLinearizations) {
    errSq += mpL->linearize(updateState);
  }
  return errSq;
}

void SqrtProblem::decomposeLinearization() {
  //YSTODO: tbb
  for (auto& mpL : mMapPointLinearizations) {
    mpL->decomposeWithQR();
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
  for (auto* param : mFrameParameters) {
    param->backup();
  }

  for (auto* param : mMapPointParameters) {
    param->backup();
  }
}
}  //namespace toy

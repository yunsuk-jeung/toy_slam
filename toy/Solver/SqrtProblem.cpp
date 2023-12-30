#include "config.h"
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
  mOption = Option();

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
  //prepare solving
  const int Hrows = mFrameParameters.size() * FrameParameter::SIZE;
  mH.resize(Hrows, Hrows);
  mB.resize(Hrows);

  auto& lambda    = mOption.mLambda;
  auto& minLambda = mOption.mMinLambda;
  auto& maxLambda = mOption.mMaxLambda;
  auto  mu        = mOption.mMu;
  auto& muFactor  = mOption.mMuFactor;

  bool terminated = false;
  bool converged  = false;

  ToyLogD("-------- Start optimize --------");
  int successfulIter = 0;
  int iter           = 0;

  while (iter <= Config::Vio::maxIteration && !terminated) {
    double currErrSq = linearize(true);
    //ToyLogD("initial err : {}", currErrSq);

    decomposeLinearization();

    mH.setZero();
    mB.setZero();

    //construct hessian for frames
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

    while (iter <= Config::Vio::maxIteration && !terminated) {
      bool            deltaValid = false;
      Eigen::VectorXd frameDelta;

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

      double newErrSq = linearize(false);

      bool validStep = true;

      if (Config::Vio::compareLinearizedDiff) {
        TOY_ASSERT_MESSAGE(false, "not implemented");
        validStep = false;
      }

      double errDiff        = currErrSq - newErrSq;
      bool   successfulStep = errDiff > 0 && validStep;

      if (successfulStep) {
        lambda *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * errDiff - 1.0, 3.0));
        lambda = std::max(minLambda, lambda);

        mu = mOption.mMu;

        ++iter;

        double stepSize = frameDelta.array().abs().maxCoeff();
        //ToyLogD("frame delta : {}", ToyLogger::eigenVec(frameDelta, 4));
        // clang-format off
        ToyLogD("iter : {:02d} {:<2} sucessed. {:<4}  {:03.2f}->{:03.2f}, frame step size : {:.4f}",
                iter,
                "",
                "error",
                currErrSq,
                newErrSq,
                stepSize);
        // clang-format on

        if ((errDiff > 0 && errDiff < 1e-6) || stepSize < 1e-4) {
          converged  = true;
          terminated = true;
        }
        ++successfulIter;
        //stop inner lm loop
        break;
      }
      else {
        // clang-format off
        ToyLogD("iter : {:02d} {:<2} failed.   {:<4} {:03.4f}->{:03.4f}",
                iter,
                "",
                "update lambda",
                lambda,
                lambda * mu);
        // clang-format on

        lambda = mu * lambda;
        mu *= muFactor;

        restoreParameters();
        ++iter;

        if (lambda > maxLambda) {
          terminated = true;
          ToyLogD("Solver did not converge and reached maximum damping lambda");
        }
      }
    }
  }

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

void SqrtProblem::restoreParameters() {
  for (auto* param : mFrameParameters) {
    param->restore();
  }

  for (auto* param : mMapPointParameters) {
    param->restore();
  }
}
}  //namespace toy

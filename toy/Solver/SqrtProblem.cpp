#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include "config.h"
#include "ToyAssert.h"
#include "DebugUtil.h"
#include "SqrtProblem.h"
#include "MapPoint.h"
#include "CostFunction.h"
#include "SqrtMarginalizationCost.h"
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
  : mFrames{nullptr}
  , mMapPoints{nullptr} {}

SqrtProblem::~SqrtProblem() {
  reset();
}

void SqrtProblem::reset() {
  mOption = Option();

  mH.setZero();
  mB.setZero();

  mFrameIdColumnMap.clear();
  mFrames    = nullptr;
  mMapPoints = nullptr;

  mMapPointLinearizations.clear();
}

void SqrtProblem::setFrames(std::vector<db::Frame::Ptr>* framesRp) {
  mFrames = framesRp;

  int column = 0;
  for (auto& frame : *mFrames) {
    mFrameIdColumnMap[frame->id()] = column;
    column += db::Frame::PARAMETER_SIZE;
  }
}

void SqrtProblem::setMapPoints(std::vector<std::shared_ptr<db::MapPoint>>* mapPointsRp) {
  mMapPoints = mapPointsRp;
}

void SqrtProblem::addReprojectionCost(db::MapPoint::Ptr                  mp,
                                      std::vector<ReprojectionCost::Ptr> costs) {
  mMapPointLinearizations.emplace_back(
    std::make_shared<MapPointLinearization>(mp, &mFrameIdColumnMap, costs));
}

void SqrtProblem::addMarginalizationCost(SqrtMarginalizationCost::Ptr cost) {
  mSqrtMarginalizationCost = cost;
}

std::vector<MapPointLinearization::Ptr> SqrtProblem::getMaPointLinearizations(
  std::vector<db::MapPoint::Ptr>& mps) {
  if (mps.empty())
    return {};

  std::vector<MapPointLinearization::Ptr> outs;
  outs.reserve(mMapPointLinearizations.size());
  auto linearizationIt = mMapPointLinearizations.begin();

  for (auto it = mps.begin(); it != mps.end(); ++it) {
    for (; linearizationIt != mMapPointLinearizations.end(); ++linearizationIt) {
      if (*it == (*linearizationIt)->mp()) {
        outs.push_back(*linearizationIt);
        linearizationIt++;
        break;
      }
    }
  }

  return outs;
}

bool SqrtProblem::solve() {
  const auto& frames = *mFrames;

  //prepare solving
  const int Hrows = frames.size() * db::Frame::PARAMETER_SIZE;
  mH.resize(Hrows, Hrows);
  mB.resize(Hrows);

  auto& lambda    = mOption.mLambda;
  auto& minLambda = mOption.mMinLambda;
  auto& maxLambda = mOption.mMaxLambda;
  auto  mu        = mOption.mMu;
  auto& muFactor  = mOption.mMuFactor;

  bool terminated = false;
  bool converged  = false;

  if (Config::Vio::solverLogDebug) {
    ToyLogD("-------- Start optimize --------");
  }

  int successfulIter = 0;
  int iter           = 0;

  while (iter <= Config::Vio::maxIteration && !terminated) {
    double currErrSq = linearize(true);
    //ToyLogD("initial err : {}", currErrSq);

    decomposeLinearization();

    mH.setZero();
    mB.setZero();

    //construct hessian for frames from mapPoints
    for (auto& mpL : mMapPointLinearizations) {
      const Eigen::MatrixXd& vJ   = mpL->J();
      const Eigen::VectorXd& vRes = mpL->Res();

      const auto  rows = vJ.rows() - db::MapPoint::PARAMETER_SIZE;
      const auto& cols = Hrows;

      const Eigen::MatrixXd J   = vJ.bottomLeftCorner(rows, cols);
      const Eigen::VectorXd Res = vRes.bottomRows(rows);

      Eigen::MatrixXd Jt = J.transpose();

      mH += Jt * J;
      mB -= Jt * Res;
    }

    //construct hessian for frames from marginCost
    mSqrtMarginalizationCost->addToHessian(mH, mB);

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
      for (auto& fp : frames) {
        fp->update(frameDelta.segment<db::Frame::PARAMETER_SIZE>(frameRow));
        frameRow += db::Frame::PARAMETER_SIZE;
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
        if (Config::Vio::solverLogDebug) {
          // clang-format off
        ToyLogD("iter : {:02d} {:<2} sucessed. {:<4}  {:03.2f}->{:03.2f}, frame step size : {:.5f}",
                iter,
                "",
                "error",
                currErrSq,
                newErrSq,
                stepSize);
          // clang-format on
        }
        if ((errDiff > 0 && errDiff < 1e-6) || stepSize < 1e-4) {
          converged  = true;
          terminated = true;
        }
        ++successfulIter;
        //stop inner lm loop
        break;
      }
      else {
        if (Config::Vio::solverLogDebug) {
          // clang-format off
        ToyLogD("iter : {:02d} {:<2} failed.   {:<4} {:03.5f}->{:03.5f}",
                iter,
                "",
                "update lambda",
                lambda,
                lambda * mu);
          // clang-format on
        }

        lambda = mu * lambda;
        mu *= muFactor;

        restoreParameters();
        ++iter;

        if (lambda > maxLambda) {
          terminated = true;
          if (Config::Vio::solverLogDebug) {
            ToyLogD("Solver did not converge and reached maximum damping lambda");
          }
        }
      }
    }
  }

  return true;
}

double SqrtProblem::linearize(bool updateState) {
  double errSq = 0;

  //if (Config::Vio::tbb) {
  //  auto sumErrorSq = [&](const tbb::blocked_range<size_t>& r, double error) {
  //    for (size_t i = r.begin(); i != r.end(); ++i) {
  //      auto& mpL = mMapPointLinearizations[i];
  //      error += mpL->linearize(updateState);
  //    }
  //    return error;
  //  };

  //  auto                       mpLinearizationSize = mMapPointLinearizations.size();
  //  tbb::blocked_range<size_t> range(0, mpLinearizationSize);
  //  errSq = tbb::parallel_reduce(range, double(0), sumErrorSq, std::plus<double>());
  //}
  //else {
    for (auto& mpL : mMapPointLinearizations) {
      errSq += mpL->linearize(updateState);
    }
  //}

  errSq += mSqrtMarginalizationCost->linearize();

  return errSq;
}

void SqrtProblem::decomposeLinearization() {
  //if (Config::Vio::tbb) {
  //  auto decompose = [&](const tbb::blocked_range<size_t>& r) {
  //    for (size_t i = r.begin(); i != r.end(); ++i) {
  //      mMapPointLinearizations[i]->decomposeWithQR();
  //    }
  //  };
  //  auto                       mpLSize = mMapPointLinearizations.size();
  //  tbb::blocked_range<size_t> range(0, mpLSize);
  //  tbb::parallel_for(range, decompose);
  //}
  //else {
    for (auto& mpL : mMapPointLinearizations) {
      mpL->decomposeWithQR();
    }
  //}
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
  for (auto& f : *mFrames) {
    f->backup();
  }

  for (auto& mp : *mMapPoints) {
    mp->backup();
  }
}

void SqrtProblem::restoreParameters() {
  for (auto& f : *mFrames) {
    f->restore();
  }
  for (auto& mp : *mMapPoints) {
    mp->restore();
  }
}
}  //namespace toy

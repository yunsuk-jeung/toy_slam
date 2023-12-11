#pragma once
#include "macros.h"
#include "usings.h"
#include "ToyLogger.h"
#include "Frame.h"
#include "MapPoint.h"
#include "EigenUtil.h"
#include "Factor.h"

namespace toy {
class MEstimator {
public:
  USING_SMART_PTR(MEstimator);
  MEstimator() = delete;
  MEstimator(double c = 1.0)
    : mC{c} {}

  virtual std::tuple<double, double> computeError(double errorSq) = 0;

protected:
  double mC;  //constant
};

class Huber : public MEstimator {
public:
  USING_SMART_PTR(Huber);
  Huber(double c = 1.0)
    : MEstimator(c) {}

  std::tuple<double, double> computeError(double errorSq) {
    const double weight = errorSq <= mC * mC ? 1.0 : mC / std::sqrt(errorSq);
    const double error  = 0.5 * (2 - weight) * weight * errorSq;
    return {error, weight};
  }
};

class PoseOnlyReprojectionCost {
public:
  PoseOnlyReprojectionCost(Sophus::SE3d*     Swc,
                           db::MapPoint::Ptr mp,
                           Eigen::Vector2d   undist,
                           MEstimator::Ptr   ME,
                           double            scale = 640.0)
    : mSwc{Swc}
    , mMapPoint{mp}
    , mJ{Eigen::Matrix26d::Zero()}
    , mC{Eigen::Vector2d::Zero()}
    , mZ{undist}
    , mME{ME}
    , mScale{scale} {}

  double linearlize() {
    Eigen::Vector3d Pwx = mMapPoint->getPwx();

    Sophus::SE3d    Scw  = mSwc->inverse();
    Eigen::Matrix3d Rcw  = Scw.rotationMatrix();
    Eigen::Vector3d Pcx  = Scw * Pwx;
    double          z    = Pcx.z();
    double          iz   = 1.0 / z;
    double          izSq = iz * iz;

    Eigen::Vector3d undist = Pcx * iz;
    mC                     = mScale * (undist.head(2) - mZ);

    Eigen::Vector2d undist2d = undist.head(2);

    Eigen::Matrix23d reduce;
    reduce << iz, 0.0, -Pcx.x() * izSq, 0.0, iz, -Pcx.y() * izSq;

    Eigen::Matrix36d J;
    J.leftCols(3)  = -Rcw;
    J.rightCols(3) = Eigen::skew(Pcx);

    mJ = mScale * reduce * J;

    double cSq    = mC.squaredNorm();
    double error  = cSq;
    double weight = 1.0;

    if (mME)
      std::tie(error, weight) = mME->computeError(cSq);

    double sqrtW = std::sqrt(weight);
    //ToyLogD("error : {} - {} = {} -> norm : {} , sqrtW {}", ToyLogger::eigenVec(undist2d),
    //ToyLogger::eigenVec(mZ), ToyLogger::eigenVec(mC), error, sqrtW);

    mC *= sqrtW;
    mJ *= sqrtW;

    return error;
  }

  void addToHessian(Eigen::Matrix66d& H, Eigen::Vector6d& b) {
    H += mJ.transpose() * mJ;
    b += mJ.transpose() * mC;
  }

protected:
  db::MapPoint::Ptr mMapPoint;
  Sophus::SE3d*     mSwc;
  Eigen::Matrix26d  mJ;  //jacobian
  Eigen::Vector2d   mC;  //cost
  Eigen::Vector2d   mZ;  //measurement

  MEstimator::Ptr mME;
  double          mScale;

  public:
  db::MapPoint::Ptr getMapPoint() {return mMapPoint;}
};
}  //namespace toy
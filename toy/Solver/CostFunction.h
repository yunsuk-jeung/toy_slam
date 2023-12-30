#pragma once
#include "macros.h"
#include "usings.h"
#include "ToyLogger.h"
#include "Frame.h"
#include "MapPoint.h"
#include "EigenUtil.h"
#include "Parameter.h"
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
    //ToyLogD("error : {} - {} = {} -> norm : {} , sqrtW {}",
    //ToyLogger::eigenVec(undist2d), ToyLogger::eigenVec(mZ), ToyLogger::eigenVec(mC),
    //error, sqrtW);

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
  db::MapPoint::Ptr getMapPoint() { return mMapPoint; }
};

class ReprojectionCost {
public:
  USING_SMART_PTR(ReprojectionCost);
  ReprojectionCost() = delete;
  ReprojectionCost(FrameParameter*    fs0,
                   Sophus::SE3d&      Tb0c0,
                   FrameParameter*    fs1,
                   Sophus::SE3d&      Tb1c1,
                   MapPointParameter* ms,
                   Eigen::Vector3d&   maesurement,
                   MEstimator::Ptr    ME,
                   double             sqrtInfo = 640.0)
    : mFP0{fs0}
    , mFP1{fs1}
    , mMpP{ms}
    , mZ{maesurement}
    , mME{ME}
    , mSqrtInfo{sqrtInfo} {
    mRb0c0 = Tb0c0.rotationMatrix();
    mPb0c0 = Tb0c0.translation();

    auto Tc1b1 = Tb1c1.inverse();
    mRc1b1     = Tc1b1.rotationMatrix();
    mPc1b1     = Tc1b1.translation();
  }

  virtual double linearlize(bool updateState) {
    const Sophus::SE3d& Twb0 = mFP0->Twb();
    const Sophus::SE3d  Tb1w = mFP1->Twb().inverse();

    const Eigen::Matrix3d Rwb0 = Twb0.rotationMatrix();
    const Eigen::Vector3d Pwb0 = Twb0.translation();
    const Eigen::Matrix3d Rb1w = Tb1w.rotationMatrix();
    const Eigen::Vector3d Pb1w = Tb1w.translation();

    const double&         invD   = mMpP->invD();
    const double          D      = 1.0 / invD;
    const auto&           undist = mMpP->undist();
    const Eigen::Vector3d undist3d(undist.x(), undist.y(), 1.0);

    Eigen::Vector3d Pc0x = undist3d * D;
    Eigen::Vector3d Pb0x = mRb0c0 * Pc0x + mPb0c0;
    Eigen::Vector3d Pb1x = Rb1w * (Rwb0 * Pb0x + Pwb0) + Pb1w;
    Eigen::Vector3d Pc1x = mRc1b1 * Pb1x + mPc1b1;

    double z  = Pc1x.z();
    double iz = 1.0 / z;

    Eigen::Vector3d nPc1x = Pc1x * iz;
    Eigen::Vector2d cost  = mSqrtInfo * (nPc1x - mZ).head(2);

    double cSq    = cost.squaredNorm();
    double errSq  = cSq;
    double weight = 1.0;

    if (mME) {
      std::tie(errSq, weight) = mME->computeError(cSq);
    }

    if (updateState) {
      double sqrtW = std::sqrt(weight);

      mC = sqrtW * cost;

      double           izSq = iz * iz;
      Eigen::Matrix23d reduce;
      reduce << iz, 0.0, -Pc1x.x() * izSq, 0.0, iz, -Pc1x.y() * izSq;

      double totalSqrtInfo = sqrtW * mSqrtInfo;

      Eigen::Matrix3d Rc1w  = mRc1b1 * Rb1w;
      Eigen::Matrix3d Rc1b0 = Rc1w * Rwb0;

      Eigen::Matrix36d J;
      J.block<3, 3>(0, 0) = Rc1w;
      J.block<3, 3>(0, 3) = -Rc1b0 * Eigen::skew(Pb0x);
      mJ_f0               = totalSqrtInfo * reduce * J;

      J.block<3, 3>(0, 0) = -Rc1w;
      J.block<3, 3>(0, 3) = mRc1b1 * Eigen::skew(Pb1x);
      mJ_f1               = totalSqrtInfo * reduce * J;

      Eigen::Matrix3d Rc1c0 = Rc1b0 * mRb0c0;
      Eigen::Matrix3d Jmp;
      Jmp.leftCols(2)  = Rc1c0.leftCols(2) * D;
      Jmp.rightCols(1) = Rc1c0 * Pc0x * -D;
      mJ_mp            = totalSqrtInfo * reduce * Jmp;
    }
    return errSq;
  }

public:
  static constexpr int SIZE = 2;

protected:
  FrameParameter* mFP0;
  FrameParameter* mFP1;

  Eigen::Matrix3d mRb0c0;
  Eigen::Vector3d mPb0c0;
  Eigen::Matrix3d mRc1b1;
  Eigen::Vector3d mPc1b1;

  MapPointParameter* mMpP;

  Eigen::Vector3d mZ;  //measurement

  MEstimator::Ptr mME;
  double          mSqrtInfo;

  Eigen::Vector2d  mC;     //cost
  Eigen::Matrix26d mJ_f0;  //jacobian for host
  Eigen::Matrix26d mJ_f1;  //jacobian for target
  Eigen::Matrix23d mJ_mp;  //jacobian for mp

public:
  FrameParameter*    getFrameParameter0() { return mFP0; }
  FrameParameter*    getFrameParameter1() { return mFP1; }
  MapPointParameter* getMapPointParameter() { return mMpP; }

  const Eigen::Vector2d&  C() const { return mC; }
  const Eigen::Matrix26d& J_f0() const { return mJ_f0; }
  const Eigen::Matrix26d& J_f1() const { return mJ_f1; }
  const Eigen::Matrix23d& J_mp() const { return mJ_mp; }
};

class StereoReprojectionCost : public ReprojectionCost {
public:
  StereoReprojectionCost() = delete;
  StereoReprojectionCost(FrameParameter*    fs0,
                         Sophus::SE3d&      Tbc0,
                         FrameParameter*    fs1,
                         Sophus::SE3d&      Tbc1,
                         MapPointParameter* ms,
                         Eigen::Vector3d&   maesurement,
                         MEstimator::Ptr    ME,
                         double             sqrtInfo = 640.0)
    : ReprojectionCost(fs0, Tbc0, fs1, Tbc1, ms, maesurement, ME, sqrtInfo) {}

  double linearlize(bool updateState) override {
    const double&         invD   = mMpP->invD();
    const double          D      = 1.0 / invD;
    const auto&           undist = mMpP->undist();
    const Eigen::Vector3d undist3d(undist.x(), undist.y(), 1.0);

    Eigen::Vector3d Pc0x = undist3d * D;

    Eigen::Matrix3d Rc1c0 = mRc1b1 * mRb0c0;
    Eigen::Vector3d Pc1c0 = mRc1b1 * mPb0c0 + mPc1b1;

    Eigen::Vector3d Pc1x = Rc1c0 * Pc0x + Pc1c0;

    double z  = Pc1x.z();
    double iz = 1.0 / z;

    Eigen::Vector3d nPc1x = Pc1x * iz;
    Eigen::Vector2d cost  = mSqrtInfo * (nPc1x - mZ).head(2);

    double cSq    = cost.squaredNorm();
    double errSq  = cSq;
    double weight = 1.0;

    if (mME) {
      std::tie(errSq, weight) = mME->computeError(cSq);
    }

    if (updateState) {
      double sqrtW = std::sqrt(weight);

      mC = sqrtW * cost;

      mJ_f0.setZero();
      mJ_f1.setZero();

      double           izSq = iz * iz;
      Eigen::Matrix23d reduce;
      reduce << iz, 0.0, -Pc1x.x() * izSq, 0.0, iz, -Pc1x.y() * izSq;

      Eigen::Matrix3d Jmp;
      Jmp.leftCols(2)  = Rc1c0.leftCols(2) * D;
      Jmp.rightCols(1) = Rc1c0 * Pc0x * -D;
      mJ_mp            = sqrtW * mSqrtInfo * reduce * Jmp;
    }
    return errSq;
  }

protected:
};

}  //namespace toy
#pragma once
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include "ImagePyramid.h"
#include "Camera.h"

#include <sophus/se3.hpp>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include "ToyLogger.h"
#include "config.h"
#include "usings.h"
#include "Frame.h"
#include "MapPoint.h"
#include "CostFunction.h"

namespace toy {
class BasicSolver {
public:
  static bool triangulate(const Eigen::Vector3d undist0,
                          const Eigen::Vector3d undist1,
                          const Sophus::SE3d&   Sc1c0,
                          Eigen::Vector3d&      out) {
    Eigen::Matrix<double, 3, 4> P1 = Eigen::Matrix<double, 3, 4>::Identity();
    Eigen::Matrix<double, 3, 4> P2 = Sc1c0.matrix3x4();
    Eigen::Matrix4d             A;
    A.row(0) = undist0[0] * P1.row(2) - undist0[2] * P1.row(0);
    A.row(1) = undist0[1] * P1.row(2) - undist0[2] * P1.row(1);
    A.row(2) = undist1[0] * P2.row(2) - undist1[2] * P2.row(0);
    A.row(3) = undist1[1] * P2.row(2) - undist1[2] * P2.row(1);

    Eigen::Vector4d homoPoint;
    homoPoint = A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    out       = homoPoint.head(3) / homoPoint.w();

    auto& min = Config::Solver::basicMinDepth;
    auto& max = Config::Solver::basicMaxDepth;

    if (out.z() < min || out.z() > max)
      return false;

    return true;
  }

  static bool solveFramePose(db::Frame::Ptr curr) {
    auto&            mapPointFactorMap = curr->getMapPointFactorMap();
    Eigen::Matrix66d H                 = Eigen::Matrix66d::Zero();
    Eigen::Vector6d  b                 = Eigen::Vector6d::Zero();

    MEstimator::Ptr huber = std::shared_ptr<Huber>(new Huber(1.0));
    //MEstimator::Ptr huber = nullptr;

    {
      Sophus::SE3d Swc = curr->getSwc(0);
      auto*        cam = curr->getCamera(0);

      MEstimator::Ptr huber = nullptr;

      std::vector<PoseOnlyReprojectionCost> costs;
      costs.reserve(mapPointFactorMap.size());

      for (auto& [mpWeak, factor] : mapPointFactorMap) {
        auto mp = mpWeak.lock();

        Sophus::SE3d    Scw       = Swc.inverse();
        cv::Point2f     uv        = cv::Point2f(factor.uv0().x(), factor.uv0().y());
        Eigen::Vector3d Pwx       = mp->getPwx();
        Eigen::Vector3d Pcx       = Scw * Pwx;
        auto            cvPoint2d = cam->project(Pcx);

        Eigen::Vector2d res = 640
                              * Eigen::Vector2d(uv.x - cvPoint2d.x, uv.y - cvPoint2d.y);
        if (res.norm() < 10)
          costs.emplace_back(&Swc, mp, factor.undist0().head(2), huber);
      }

      Eigen::Matrix66d JtJ = Eigen::Matrix66d::Zero();
      Eigen::Vector6d  JtC = Eigen::Vector6d::Zero();
      double           err = 0;
      for (auto& cost : costs) {
        err += cost.linearlize();
        //cost.addToHessian(JtJ, JtC);
      }
      std::cout << err << std::endl;

      double* ptr = Swc.data();
      ptr[4] += 0.1;
      ToyLogI("WTF1 : {}", ToyLogger::se3String(Swc));

      err = 0;
      for (auto& cost : costs) {
        err += cost.linearlize();
        std::cout << cost.getMapPoint()->id() << " "
                  << "err " << err << std::endl;

        //cost.addToHessian(JtJ, JtC);
      }
      std::cout << err << std::endl;
    }

    std::vector<PoseOnlyReprojectionCost> costs;
    costs.reserve(mapPointFactorMap.size());
    Sophus::SE3d Swc = curr->getSwc(0);
    ToyLogI("Start : {}", ToyLogger::se3String(Swc));

    cv::Mat image = curr->getImagePyramid(0)->getOrigin().clone();
    cv::cvtColor(image, image, CV_GRAY2BGR);
    auto  Scw = Swc.inverse();
    auto* cam = curr->getCamera(0);
    for (auto& [mpWeak, factor] : mapPointFactorMap) {
      auto mp = mpWeak.lock();
      //costs.emplace_back(&Swc, mp, factor.undist0().head(2), huber);

      cv::Point2f uv = cv::Point2f(factor.uv0().x(), factor.uv0().y());
      cv::circle(image, uv, 6, {255, 0, 0}, -1);

      auto cvPoint2d2 = cam->project(factor.undist0());
      cv::circle(image, cvPoint2d2, 5, {0, 255, 0}, -1);

      Eigen::Vector3d Pwx       = mp->getPwx();
      Eigen::Vector3d Pcx       = Scw * Pwx;
      auto            cvPoint2d = cam->project(Pcx);
      cv::circle(image, cvPoint2d, 4, {0, 0, 255}, -1);

      Eigen::Vector2d res = 640 * Eigen::Vector2d(uv.x - cvPoint2d.x, uv.y - cvPoint2d.y);
      //if (res.norm() < 20)
      costs.emplace_back(&Swc, mp, factor.undist0().head(2), huber);
    }

    cv::imshow("before opt", image);
    cv::waitKey(1);

    /*
    //single thread is faster for the small problem
    struct Reductor {
      Reductor(std::vector<PoseOnlyReprojectionCost>& costs)
        : mCosts{costs} {
        mH.setZero();
        mb.setZero();
      }
      void operator()(const tbb::blocked_range<size_t>& range) {
        for (size_t r = range.begin(); r != range.end(); ++r) {
          auto& cost = mCosts[r];
          cost.linearlize();
          cost.addToHessian(mH, mb);
        }
      }
      Reductor(Reductor& src, tbb::split)
        : mCosts(src.mCosts) {
        mH.setZero();
        mb.setZero();
      };
      inline void join(Reductor& b) {
        mH += b.mH;
        mb += b.mb;
      }

    std::vector<PoseOnlyReprojectionCost>& mCosts;
    Eigen::Matrix6d                        mH;
    Eigen::Vector6d                        mb;
    };

    Reductor                   reductor(costs);
    tbb::blocked_range<size_t> range(0, costs.size());
    tbb::parallel_reduce(range, reductor);
    */
    ToyLogI("Before 1: {}", ToyLogger::se3String(Swc));

    double* ptr = Swc.data();
    ptr[4] += 0.1;
    ToyLogI("Before 2: {}", ToyLogger::se3String(Swc));
    constexpr int    maxIter = 2;
    constexpr double lambda  = 1e-6;

    for (int iter = 0; iter < 1; iter++) {
      Eigen::Matrix66d JtJ = Eigen::Matrix66d::Zero();
      Eigen::Vector6d  JtC = Eigen::Vector6d::Zero();

      double err = 0.0;
      for (auto& cost : costs) {
        err += cost.linearlize();

        std::cout << cost.getMapPoint()->id() << " "
                  << "err " << err << std::endl;
        cost.addToHessian(JtJ, JtC);
      }

      //ToyLogD("JTJ : {}", ToyLogger::eigenMat(JtJ));
      Eigen::Vector6d D = JtJ.diagonal();
      D *= lambda;
      JtJ.diagonal().array() += D.array().max(lambda);

      //ToyLogD("JTJ : {}", ToyLogger::eigenMat(JtJ));
      //ToyLogD("JtC : {}", ToyLogger::eigenVec(JtC));

      Eigen::Vector6d delX = -JtJ.ldlt().solve(JtC);

      ToyLogD("error Square : {}, del X : {}", err, ToyLogger::eigenVec(delX));

      Swc = Swc * Sophus::SE3d::exp(delX);
      ToyLogI("After : {}", ToyLogger::se3String(Swc));
    }

    auto&        Sbc0 = curr->getSbc(0);
    Sophus::SE3d Swb  = Swc * Sbc0.inverse();

    curr->setSwb(Swb);
    {
      cv::Mat image = curr->getImagePyramid(0)->getOrigin().clone();
      cv::cvtColor(image, image, CV_GRAY2BGR);
      Sophus::SE3d Swc = curr->getSwc(0);
      auto         Scw = Swc.inverse();
      auto*        cam = curr->getCamera(0);
      for (auto& [mpWeak, factor] : mapPointFactorMap) {
        auto mp = mpWeak.lock();
        costs.emplace_back(&Swc, mp, factor.undist0().head(2), huber);

        cv::Point2f uv = cv::Point2f(factor.uv0().x(), factor.uv0().y());
        cv::circle(image, uv, 6, {255, 0, 0}, -1);

        auto cvPoint2d2 = cam->project(factor.undist0());
        cv::circle(image, cvPoint2d2, 5, {0, 255, 0}, -1);

        Eigen::Vector3d Pwx       = mp->getPwx();
        Eigen::Vector3d Pcx       = Scw * Pwx;
        auto            cvPoint2d = cam->project(Pcx);
        cv::circle(image, cvPoint2d, 4, {0, 0, 255}, -1);
      }
      cv::imshow("after opt", image);
      cv::waitKey();
    }
    return true;
  }
};
}  //namespace toy

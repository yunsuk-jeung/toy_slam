#include "ToyLogger.h"
#include "config.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Factor.h"
#include "CostFunction.h"
#include "SqrtProblem.h"
#include "SqrtLocalSolver.h"

namespace toy {
namespace {
MEstimator::Ptr createReprojectionMEstimator() {
  switch (Config::Vio::reprojectionME) {
  case 1: {
    return std::make_shared<Huber>(Config::Vio::reprojectionMEConst);
    break;
  }
  case 0:
  default:
    break;
  }
  return nullptr;
}
}  //namespace

SqrtLocalSolver::SqrtLocalSolver() {}

SqrtLocalSolver::~SqrtLocalSolver() {}

bool SqrtLocalSolver::solve(std::vector<db::Frame::Ptr>&    frames,
                            std::vector<db::MapPoint::Ptr>& mapPoints) {
  if (frames.size() < 4)
    return false;

  createStates(frames, mapPoints);

  SqrtProblem::Uni problem = std::make_unique<SqrtProblem>();
  /*problem->mOption.mLambda;*/

  problem->setFrameSatatesMap(&mFrameParameterMap);
  problem->setMapPointState(&mMapPointParameterMap);

  MEstimator::Ptr reProjME    = createReprojectionMEstimator();
  const double&   focalLength = Config::Vio::standardFocalLength;

  for (auto& mp : mapPoints) {
    auto& frameFactors = mp->getFrameFactors();

    std::vector<ReprojectionCost::Ptr> costs;
    costs.reserve(frameFactors.size());

    auto it  = frameFactors.begin();
    auto end = frameFactors.end();

    db::Frame::Ptr frame0 = (*it).first.lock();
    Sophus::SE3d&  Tbc0   = frame0->getTbc(0);

    auto* mpParam  = &mMapPointParameterMap[mp->id()];
    auto* frParam0 = &mFrameParameterMap[frame0->id()];

    if (it->second.getType() == db::ReprojectionFactor::Type::STEREO) {
      Sophus::SE3d&    Tbc1    = frame0->getTbc(1);
      Eigen::Vector3d& undist1 = it->second.undist1();

      ReprojectionCost::Ptr cost = std::make_shared<StereoReprojectionCost>(frParam0,
                                                                            Tbc0,
                                                                            frParam0,
                                                                            Tbc1,
                                                                            mpParam,
                                                                            undist1,
                                                                            reProjME,
                                                                            focalLength);
      costs.push_back(cost);
    }

    ++it;
    for (; it != end; ++it) {
      db::Frame::Ptr   frame1   = (*it).first.lock();
      auto*            frParam1 = &mFrameParameterMap[frame1->id()];
      Sophus::SE3d&    Tbc1     = frame1->getTbc(0);
      Eigen::Vector3d& undist0  = it->second.undist0();

      ReprojectionCost::Ptr cost = std::make_shared<ReprojectionCost>(frParam0,
                                                                      Tbc0,
                                                                      frParam1,
                                                                      Tbc1,
                                                                      mpParam,
                                                                      undist0,
                                                                      reProjME,
                                                                      focalLength);
      costs.push_back(cost);

      if (it->second.getType() != db::ReprojectionFactor::Type::STEREO)
        continue;

      Sophus::SE3d&    Tbc2    = frame1->getTbc(1);
      Eigen::Vector3d& undist1 = it->second.undist1();

      ReprojectionCost::Ptr cost2 = std::make_shared<ReprojectionCost>(frParam0,
                                                                       Tbc0,
                                                                       frParam1,
                                                                       Tbc2,
                                                                       mpParam,
                                                                       undist1,
                                                                       reProjME,
                                                                       focalLength);
      costs.push_back(cost2);
    }

    problem->addReprojectionCost(mp, costs);
  }

  auto result = problem->solve();

  return result;
}

void SqrtLocalSolver::marginalize(int id) {
  ToyLogE("NOT IMPLEMENTED YET, {}", __FUNCTION__);
}

void SqrtLocalSolver::createStates(std::vector<db::Frame::Ptr>&    frames,
                                   std::vector<db::MapPoint::Ptr>& mapPoints) {
  mFrameParameterMap.clear();
  mMapPointParameterMap.clear();

  for (auto& f : frames) {
    mFrameParameterMap.insert({f->id(), FrameParameter(f)});
  }

  for (auto& mp : mapPoints) {
    mMapPointParameterMap.insert({mp->id(), MapPointParameter(mp)});
  }
}

}  //namespace toy
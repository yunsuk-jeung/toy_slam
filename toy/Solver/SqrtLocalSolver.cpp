#include <numeric>
#include <unordered_set>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include "ToyLogger.h"
#include "config.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "Camera.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Factor.h"
#include "SqrtMarginalizationCost.h"
#include "CostFunction.h"
#include "SqrtMarginalizer.h"
#include "SqrtMarginalizationCost.h"
#include "MapPointLinearization.h"
#include "SqrtProblem.h"
#include "SqrtLocalSolver.h"

namespace toy {
namespace {
size_t MP_SIZE    = db::MapPoint::PARAMETER_SIZE;
size_t FRAME_SIZE = db::Frame::PARAMETER_SIZE;

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

SqrtLocalSolver::SqrtLocalSolver() {
  mProblem      = std::make_unique<SqrtProblem>();
  mMarginalizer = std::make_unique<SqrtMarginalizer>();

  const size_t& initialFrameSize = Config::Vio::solverMinimumFrames - 1;
  const auto    cols             = FRAME_SIZE * initialFrameSize;

  Eigen::MatrixXd& J   = mMarginalizer->getJ();
  Eigen::VectorXd& Res = mMarginalizer->getRes();

  J.resize(FRAME_SIZE, cols);
  Res.resize(FRAME_SIZE);

  J.setIdentity();
  J *= 10000.0;

  Res.setZero();
}

SqrtLocalSolver::~SqrtLocalSolver() {}

bool SqrtLocalSolver::solve(const std::vector<db::Frame::Ptr>&    frames,
                            const std::vector<db::MapPoint::Ptr>& trackingMapPoints) {
  if (frames.size() < Config::Vio::solverMinimumFrames) {
    mMarginalizer->setFrames(frames);
    return false;
  }

  mFrames    = &frames;
  mMapPoints = &trackingMapPoints;

  for (auto& f : *mFrames) {
    if (!f->isLinearized()) {
      f->resetDelta();
    }
  }

  /*problem->mOption.mLambda;*/
  mProblem->reset();
  mProblem->setFrames(mFrames);
  mProblem->setMapPoints(mMapPoints);

  //reprojection cost
  MEstimator::Ptr reProjME       = createReprojectionMEstimator();
  const double&   stdFocalLength = Config::Vio::standardFocalLength;

  for (auto& mp : *mMapPoints) {
    auto& frameFactors = mp->frameFactorMap();

    std::vector<ReprojectionCost::Ptr> costs;
    costs.reserve(frameFactors.size());

    db::Frame::Ptr frame0 = mp->hostFrame();
    Sophus::SE3d&  Tbc0   = frame0->getTbc(0);

    for (auto& [frameCamId, factor] : frameFactors) {
      db::Frame::Ptr frame1 = factor.frame();
      auto&          camId  = frameCamId.camId;
      Sophus::SE3d&  Tbc1   = frame1->getTbc(camId);
      auto&          undist = factor.undist();

      ReprojectionCost::Ptr cost = std::make_shared<ReprojectionCost>(frame0,
                                                                      Tbc0,
                                                                      frame1,
                                                                      Tbc1,
                                                                      mp,
                                                                      undist,
                                                                      reProjME,
                                                                      stdFocalLength);
      costs.push_back(cost);
    }

    mProblem->addReprojectionCost(mp, costs);
  }

  auto marginCost = mMarginalizer->createMarginCost();
  mProblem->addMarginalizationCost(marginCost);

  auto result = mProblem->solve();

  return result;
}

void SqrtLocalSolver::marginalize(std::set<int64_t>& marginalkeyFrameIds,
                                  std::forward_list<db::MapPoint::Ptr>& lostMapPoints) {
  if (marginalkeyFrameIds.empty())
    return;

  std::vector<db::MapPoint::Ptr> marginalMapPoints;
  marginalMapPoints.reserve(mMapPoints->size());

  std::unordered_set<int64_t> frameIds;
  auto                        frames = mMarginalizer->frames();
  for (auto& f : frames) {
    frameIds.insert(f->id());
  }

  for (auto& mp : *mMapPoints) {
    if (marginalkeyFrameIds.count(mp->hostFrame()->id())) {
      if (mp->frameFactorMap().size() > 1) {
        marginalMapPoints.push_back(mp);
      }
    }
  }

  //for (auto& mp : lostMapPoints) {
  //  if (mp->frameFactorMap().size() > 1) {
  //    marginalMapPoints.push_back(mp);
  //  }
  //}

  for (auto& mp : marginalMapPoints) {
    auto& factorMap = mp->frameFactorMap();
    for (auto& [frameCamId, factor] : factorMap) {
      if (frameIds.count(frameCamId.frameId) == 0) {
        frames.push_back(factor.frame());
        frameIds.insert(frameCamId.frameId);
      }
    }
  }

  SqrtProblem problem;
  problem.setFrames(&frames);
  problem.setMapPoints(&marginalMapPoints);

  MEstimator::Ptr reProjME       = createReprojectionMEstimator();
  const double&   stdFocalLength = Config::Vio::standardFocalLength;
  for (auto& mp : marginalMapPoints) {
    auto&                              factorMap = mp->frameFactorMap();
    std::vector<ReprojectionCost::Ptr> costs;
    costs.reserve(factorMap.size());

    db::Frame::Ptr frame0 = mp->hostFrame();
    Sophus::SE3d&  Tbc0   = frame0->getTbc(0);

    for (auto& [frameCamId, factor] : factorMap) {
      if (frameIds.count(frameCamId.frameId) == 0) {
        frames.push_back(factor.frame());
        frameIds.insert(frameCamId.frameId);
      }
      db::Frame::Ptr frame1 = factor.frame();
      auto&          camId  = factor.camIdx();
      Sophus::SE3d&  Tbc1   = frame1->getTbc(camId);
      auto&          undist = factor.undist();

      ReprojectionCost::Ptr cost = std::make_shared<ReprojectionCost>(frame0,
                                                                      Tbc0,
                                                                      frame1,
                                                                      Tbc1,
                                                                      mp,
                                                                      undist,
                                                                      reProjME,
                                                                      stdFocalLength);
      costs.push_back(cost);
    }
    problem.addReprojectionCost(mp, costs);
  }

  problem.linearize(true);
  problem.decomposeLinearization();

  size_t     rows = 0u;
  const auto cols = frames.size() * db::Frame::PARAMETER_SIZE;

  auto mpLinearizations = mProblem->mapPointLinearization();

  for (auto& linearization : mpLinearizations) {
    rows += (linearization->J().rows() - MP_SIZE);
  }
  /*  imu factor */

  /* marginal factor*/
  rows += mMarginalizer->J().rows();
  ToyLogD("------------- margi ------------ ");
  ToyLogD("{}", mMarginalizer->J());
  /*  construct entire Jacob  */
  Eigen::MatrixXd Q2t_J;
  Eigen::VectorXd Q2t_C;

  Q2t_J.resize(rows, cols);
  Q2t_C.resize(rows);
  Q2t_J.setZero();
  Q2t_C.setZero();

  size_t currRow = 0;
  for (auto& linearization : mpLinearizations) {
    auto blockRow = linearization->J().rows() - MP_SIZE;
    // clang-format off
    auto& Q2t_J_block = 
    Q2t_J.block(currRow, 0, blockRow, cols) 
      = linearization->J().bottomLeftCorner(blockRow, cols);
    Q2t_C.segment(currRow, blockRow) = linearization->Res().tail(blockRow);
    // clang-format on
    currRow += blockRow;
  }

  {
    auto& J   = mMarginalizer->getJ();
    auto& Res = mMarginalizer->getRes();

    Q2t_J.block(currRow, 0, J.rows(), J.cols()) = J;
    Q2t_C.segment(currRow, J.rows())            = Res;
    currRow += J.rows();
  }

  std::map<int64_t, size_t>& frameColumnMap = problem.getFrameIdColumnMap();

  std::set<int> marginIndces;
  std::set<int> keepIndices;

  for (auto& f : frames) {
    const auto& id    = f->id();
    auto        start = frameColumnMap[id];
    if (marginalkeyFrameIds.count(id)) {
      for (int i = 0; i < db::Frame::PARAMETER_SIZE; ++i) {
        marginIndces.emplace(start + i);
      }
    }
    else {
      for (int i = 0; i < db::Frame::PARAMETER_SIZE; ++i) {
        keepIndices.emplace(start + i);
      }
    }
  }

  ToyLogD("margin : {} keep : {}", marginIndces.size(), keepIndices.size());

  Eigen::VectorXd delta(cols - marginIndces.size());
  Eigen::Index    deltaIdx = 0u;

  std::vector<db::Frame::Ptr> remainFrames;

  for (auto& f : frames) {
    if (!f->isLinearized()) {
      f->setLinearized(true);
    }

    if (marginalkeyFrameIds.count(f->id())) {
      continue;
    }
    remainFrames.push_back(f);
    delta.segment(deltaIdx, FRAME_SIZE) = f->getDelta();
    deltaIdx += db::Frame::PARAMETER_SIZE;
  }
  mMarginalizer->marginalize(marginIndces, keepIndices, Q2t_J, Q2t_C, delta);
  mMarginalizer->setFrames(remainFrames);
}

}  //namespace toy
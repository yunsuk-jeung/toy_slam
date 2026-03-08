#include "SLAMInfo.h"

namespace toy {
SLAMInfo::SLAMInfo()
  : mLocalPoints{}
  , mHaveLocalPoint{false}
  , mLocalPath{}
  , mHaveLocalPath{false} {}

SLAMInfo::~SLAMInfo() {}

void SLAMInfo::setLocalPoints(std::vector<float>& pts) {
  std::unique_lock<std::mutex> lock(mLocalPointLock);
  mLocalPoints.swap(pts);
  mHaveLocalPoint = true;
}

bool SLAMInfo::getLocalPoints(std::vector<float>& out) {
  std::unique_lock<std::mutex> lock(mLocalPointLock);
  if (!mHaveLocalPoint) {
    return false;
  }
  out.swap(mLocalPoints);
  mHaveLocalPoint = false;

  return true;
}

void SLAMInfo::setLocalPath(std::vector<Eigen::Matrix4f>& path) {
  std::unique_lock<std::mutex> lock(mLocalPathLock);
  mLocalPath.swap(path);
  mHaveLocalPath = true;
}

bool SLAMInfo::getLocalPath(std::vector<Eigen::Matrix4f>& out) {
  std::unique_lock<std::mutex> lock(mLocalPathLock);
  if (!mHaveLocalPath) {
    return false;
  }
  out.swap(mLocalPath);
  mHaveLocalPath = false;
  return true;
}

}  //namespace toy
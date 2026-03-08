#pragma once
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include "Singleton.h"

namespace toy {
class SLAMInfo : public Singleton<SLAMInfo> {
public:
  friend class Singleton<SLAMInfo>;

  void setLocalPoints(std::vector<float>& points);
  bool getLocalPoints(std::vector<float>& out);

  void setLocalPath(std::vector<Eigen::Matrix4f>& paths);
  bool getLocalPath(std::vector<Eigen::Matrix4f>& out);

  //void getMwc(float* Pwc);

private:
  SLAMInfo();
  ~SLAMInfo() override;

  std::mutex         mLocalPointLock;
  std::vector<float> mLocalPoints;
  bool               mHaveLocalPoint;

  std::mutex                   mLocalPathLock;
  std::vector<Eigen::Matrix4f> mLocalPath;
  bool                         mHaveLocalPath;
};
}  //namespace toy
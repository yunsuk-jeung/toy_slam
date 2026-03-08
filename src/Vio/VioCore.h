#pragma once
#include <memory>

namespace toy {
namespace db {
class ImagePyramidSet;
}  //namespace db
class FrameTracker;
class LocalTracker;
class VioCore {
public:
  VioCore();
  ~VioCore();

  void insert(std::shared_ptr<db::ImagePyramidSet> imagePyramids);
  void prepare();

  void processSync();

private:
  FrameTracker* mFrameTracker;
  LocalTracker* mLocalTracker;
};
}  //namespace toy
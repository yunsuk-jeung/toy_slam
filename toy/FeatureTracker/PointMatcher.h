#pragma once
#include <string>
namespace toy {
namespace db {
class ImagePyramid;
class Feature;
}  //namespace db
class PointMatcher {
public:
  PointMatcher(std::string type);
  ~PointMatcher();

  int process(db::ImagePyramid* imagePyramid0,
              db::Feature*      feature0,
              db::ImagePyramid* imagePyramid1,
              db::Feature*      feature1);
};
}  //namespace toy
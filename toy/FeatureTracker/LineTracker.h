#pragma once
#include <string>
#include <vector>

namespace toy {
class Camera;
namespace db {
class Feature;
class Frame;
}  //namespace db
class LineTracker {
public:
  LineTracker(std::string type) {}
  ~LineTracker() {}
};
}  //namespace toy
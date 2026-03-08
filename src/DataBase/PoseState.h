#pragma once
#include <vector>
#include <memory>
namespace toy {
namespace db {
class Frame;
class PoseState {
public:

protected:
  std::vector<std::shared_ptr<Frame>> frames_;

};
}  //namespace db
}  //namespace toy
#pragma once
#include <memory>

#include <Eigen/Dense>

namespace toy {
namespace db {
template <typename FLOAT>
class Frame {
  using Ptr = std::shared_ptr<Frame>;

public:

protected:
  Frame();
  ~Frame();

protected:
  Eigen::Vector<FLOAT, 3> wTc;
};
}  //namespace db
}  //namespace toy
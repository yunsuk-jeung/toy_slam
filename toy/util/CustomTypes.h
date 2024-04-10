#pragma once

#include <bitset>
#include <cstdint>
#include <Eigen/Dense>
#include "ToyHash.h"

namespace Eigen {
using Matrix3Pf = Eigen::Matrix<float, 3, 52>;
using MatrixP3f = Eigen::Matrix<float, 52, 3>;
using Matrix2Pf = Eigen::Matrix<float, 2, 52>;
using VectorPf  = Eigen::Matrix<float, 52, 1>;
using Matrix66d = Eigen::Matrix<double, 6, 6>;
using Matrix26d = Eigen::Matrix<double, 2, 6>;
using Matrix62d = Eigen::Matrix<double, 6, 2>;
using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;
using Vector6d  = Eigen::Matrix<double, 6, 1>;
}  //namespace Eigen

namespace toy {

struct FrameCamId {
  FrameCamId()
    : frameId(0)
    , camId(0) {}

  FrameCamId(const size_t& frameId, const size_t& camId)
    : frameId(frameId)
    , camId(camId) {}

  size_t frameId;
  size_t camId;
};

inline std::ostream& operator<<(std::ostream& os, const FrameCamId& tcid) {
  os << tcid.frameId << "_" << tcid.camId;
  return os;
}

inline bool operator<(const FrameCamId& o1, const FrameCamId& o2) {
  if (o1.frameId == o2.frameId)
    return o1.camId < o2.camId;
  return o1.frameId < o2.frameId;
}

inline bool operator==(const FrameCamId& o1, const FrameCamId& o2) {
  return o1.frameId == o2.frameId && o1.camId == o2.camId;
}

inline bool operator!=(const FrameCamId& o1, const FrameCamId& o2) {
  return o1.frameId != o2.frameId || o1.camId != o2.camId;
}

}  //namespace toy

namespace std {

template <>
struct hash<toy::FrameCamId> {
  size_t operator()(const toy::FrameCamId& x) const {
    size_t seed = 0;
    toy::combineHash(seed, x.frameId);
    toy::combineHash(seed, x.camId);
    return seed;
  }
};
}  //namespace std

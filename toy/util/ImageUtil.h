#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "ToyAssert.h"

namespace toy {
namespace util {

template <typename T>
inline bool inBounds(const cv::Mat& in, const T& x, const T& y, T border) {
  auto& w = in.cols;
  auto& h = in.rows;
  return border <= x && x < (w - border - 1) && border <= y && y < (h - border - 1);
}

template <typename T, typename N>
inline bool inBounds(const cv::Mat& in, const Eigen::Matrix<T, 2, 1>& uv, N border = 0) {
  return inBounds(in, uv.x(), uv.y(), (T)border);
}

inline void clamp(const cv::Mat& in,
                  int&           ixm1,
                  int&           ixp1,
                  int&           ixp2,
                  int&           iym1,
                  int&           iyp1,
                  int&           iyp2) {
  auto& w = in.cols;
  auto& h = in.rows;

  if (iym1 < 0)
    iym1 = 0;
  if (ixm1 < 0)
    ixm1 = 0;

  if (ixp1 > w - 1)
    ixp1 = w - 1;
  if (ixp2 > w - 1)
    ixp2 = w - 1;
  if (iyp1 > h - 1)
    iyp1 = h - 1;
  if (iyp2 > h - 1)
    iyp2 = h - 1;
}

enum InterpolateMethod { HERMITE, LINEAR };

template <typename FLOAT = float>
inline void CubHermiteSpline(const FLOAT& p0,
                             const FLOAT& p1,
                             const FLOAT& p2,
                             const FLOAT& p3,
                             const FLOAT  x,
                             FLOAT*       f,
                             FLOAT*       dfdx = nullptr) {
  const FLOAT a = 0.5 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3);
  const FLOAT b = 0.5 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3);
  const FLOAT c = 0.5 * (-p0 + p2);
  const FLOAT d = p1;
  //Horner scheme for: f = a x^3 + b x^2+ c x+d
  if (f != NULL) {
    *f = d + x * (c + x * (b + x * a));
  }

  //dfdx = 3a x^2 + 2b x + c
  if (dfdx != NULL) {
    *dfdx = c + x * (2.0 * b + 3.0 * a * x);
  }
}

template <typename FLOAT = float, typename T = float>
Eigen::Vector3f interpolateHermite(const cv::Mat& in, const Eigen::Vector<T, 2>& uv) {
  TOY_ASSERT(inBounds(in, uv, 0));
  TOY_ASSERT(in.channels() == 1);
  Eigen::Matrix<FLOAT, 3, 1> res;

  FLOAT x = uv.x();
  FLOAT y = uv.y();
  //get integer x (column) and integer y (row)
  int ix = x ;
  int iy = y ;
  //p0,1,2,3 are f(-1), f(0), f(1), f(2) at pixel position
  FLOAT p0, p1, p2, p3;
  //f0,...f3 to store function values at subpixel position
  FLOAT f0, f1, f2, f3;
  //dfdx for horizontal interpolation along each row at subpixel
  FLOAT df0dx, df1dx, df2dx, df3dx;

  //get x-1, x+1, x+2
  int ixm1 = ix - 1;
  int ixp1 = ix + 1;
  int ixp2 = ix + 2;
  //get y-1, y+1, y+2
  int iym1 = iy - 1;
  int iyp1 = iy + 1;
  int iyp2 = iy + 2;
  //to make it consistent with Ceres Interpolator
  //we clamp the pixel values beyond image border
  clamp(in, ixm1, ixp1, ixp2, iym1, iyp1, iyp2);

  //row 0
  p0 = in.at<uchar>(iym1, ixm1);
  p1 = in.at<uchar>(iym1, ix);
  p2 = in.at<uchar>(iym1, ixp1);
  p3 = in.at<uchar>(iym1, ixp2);
  CubHermiteSpline(p0, p1, p2, p3, x - ix, &f0, &df0dx);

  //row 1
  p0 = in.at<uchar>(iy, ixm1);
  p1 = in.at<uchar>(iy, ix);
  p2 = in.at<uchar>(iy, ixp1);
  p3 = in.at<uchar>(iy, ixp2);
  CubHermiteSpline(p0, p1, p2, p3, x - ix, &f1, &df1dx);

  //row 2
  p0 = in.at<uchar>(iyp1, ixm1);
  p1 = in.at<uchar>(iyp1, ix);
  p2 = in.at<uchar>(iyp1, ixp1);
  p3 = in.at<uchar>(iyp1, ixp2);
  CubHermiteSpline(p0, p1, p2, p3, x - ix, &f2, &df2dx);

  //row 3
  p0 = in.at<uchar>(iyp2, ixm1);
  p1 = in.at<uchar>(iyp2, ix);
  p2 = in.at<uchar>(iyp2, ixp1);
  p3 = in.at<uchar>(iyp2, ixp2);
  CubHermiteSpline(p0, p1, p2, p3, x - ix, &f3, &df3dx);

  //now, interpolate vertically
  CubHermiteSpline(f0, f1, f2, f3, y - iy, &res[0], &res[2]);
  CubHermiteSpline(df0dx, df1dx, df2dx, df3dx, y - iy, &res[1]);

  return res;
}

template <typename T = float>
Eigen::Vector3f interpolateLinear(const cv::Mat& in, const Eigen::Vector<T, 2>& uv) {
  TOY_ASSERT_MESSAGE(0, "not implemented");
  throw std::runtime_error("function not implemented");
}

template <typename T = float>
Eigen::Vector3f interpolateGradient(const cv::Mat&             in,
                                    const Eigen::Vector<T, 2>& uv,
                                    InterpolateMethod          method) {
  switch (method) {
  case toy::util::HERMITE:
    return interpolateHermite<T>(in, uv);
  case toy::util::LINEAR:
    return interpolateLinear<T>(in, uv);
  default:
    throw std::runtime_error("function not implemented");
  }
}

template <typename T = float>
float interpolate(const cv::Mat& in, const Eigen::Vector<T, 2>& uv) {
  auto& x = uv[0];
  auto& y = uv[1];

  int ix = x;
  int iy = y;

  T dx = x - ix;
  T dy = y - iy;

  T ddx = T(1.0) - dx;
  T ddy = T(1.0) - dy;
  // clang-format off
  return ddx * ddy * in.at<uchar>(iy    , ix) 
       + ddx * dy  * in.at<uchar>(iy + 1, ix)
       + dx  * ddy * in.at<uchar>(iy    , ix + 1) 
       + dx  * dy  * in.at<uchar>(iy + 1, ix + 1);
  // clang-format on
}

};  //namespace util
};  //namespace toy
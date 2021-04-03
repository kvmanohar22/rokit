#ifndef _RK_COMMON_GLOBAL_H_
#define _RK_COMMON_GLOBAL_H_

#include <vector>
#include <iostream>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <math.h>

#include <eigen3/Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>

#include <ceres/ceres.h>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

namespace rokit {

  using namespace std;
  using Sophus::SE3d;

  /// some useful typedefs
  typedef Eigen::Matrix<double, 2, 1> Vector2d;
  typedef Eigen::Matrix<double, 3, 1> Vector3d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  typedef Eigen::Matrix<int   , 2, 1> Vector2i;
  typedef Eigen::Matrix<int   , 3, 1> Vector3i;

  typedef Eigen::Matrix<double, 2, 2> Matrix2d;
  typedef Eigen::Matrix<double, 3, 3> Matrix3d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  typedef Eigen::Matrix<double, 2, 3> Matrix23d;

  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixNd;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1>              VectorNd;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 2>              MatrixN2d;

  static double constexpr PI = 3.1415926535;

  // TODO: Move this someplace else?
  enum class JacType
  {
    ANALYTIC,
    NUMERIC
  };

  typedef SE3d SE3;

  // TODO: need to move this somewhere else
  static constexpr int HALF_PATCH_SIZE = 2;
  static constexpr int PATCH_SIZE = 2*HALF_PATCH_SIZE;
  static constexpr int PATCH_AREA = PATCH_SIZE*PATCH_SIZE;

  // TODO: Hmm. not the best way to represent
  typedef Eigen::Matrix<double, PATCH_AREA, 1> ImgRefType;
  typedef Eigen::Matrix<double, PATCH_AREA, 2> ImgJacxyType;  /* jac of img wrt uv    */
  typedef Eigen::Matrix<double, PATCH_AREA, 6> ImgJacType;    /* jac of img wrt SE(3) */

  typedef vector<cv::Mat> ImgPyr;
} // namespace rokit

#endif // _RK_COMMON_GLOBAL_H_

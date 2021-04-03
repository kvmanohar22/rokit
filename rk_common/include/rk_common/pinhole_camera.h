#ifndef _RK_COMMON_PINHOLE_CAMERA_H_
#define _RK_COMMON_PINHOLE_CAMERA_H_

#include "rk_common/global.h"

namespace rokit {

/// Basic pinhole camera with plumb-bob (radial+tangential) distortion model
class PinholeCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(
      double fx, double fy,
      double cx, double cy,
      double k1, double k2,
      double p1, double p2, double k3,
      double width, double height
      );
  virtual ~PinholeCamera() {}

  /// map a 3D point in camera frame to image
  Vector2d cam2img(const Vector3d& xyz) const;
  Vector2d cam2img(const Vector2d& uv) const;;

  /// image to camera frame (point on the unit sphere)
  Vector3d img2cam(const Vector2d& uv) const;

  /// undistort the entire image
  void undistortImg(const cv::Mat& in, cv::Mat& out) const;

  inline double fx() const { return fx_; }
  inline double fy() const { return fy_; }
  inline double cx() const { return cx_; }
  inline double cy() const { return cy_; }

private:
  const double fx_, fy_;        //!< focal length
  const double cx_, cy_;        //!< camera center
  const double k1_, k2_, k3_;   //!< radial distortion parameters
  const double p1_, p2_;        //!< tangential distortion parameters
  bool         distortion_;     //!< distortion present?
  cv::Mat      undistort_map1_; //!< used for rectifying
  cv::Mat      undistort_map2_; //!< used for rectifying
  cv::Mat      cvK_;
  cv::Mat      cvD_;
  const double width_;
  const double height_;
}; // class PinholeCamera

} // namespace rokit

#endif // _RK_COMMON_PINHOLE_CAMERA_H_

#include "rk_common/pinhole_camera.h"
#include "rk_common/params_helper.h"
#include <opencv2/imgproc.hpp>

namespace rokit {

PinholeCamera::PinholeCamera(
      double fx, double fy,
      double cx, double cy,
      double k1, double k2,
      double p1, double p2, double k3,
      double width, double height)
  : fx_(fx), fy_(fy),
    cx_(cx), cy_(cy),
    k1_(k1), k2_(k2),
    p1_(p1), p2_(p2), k3_(k3),
    distortion_(std::abs(k1_) > 1e-3),
    width_(width), height_(height)
{
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<float>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  cv::initUndistortRectifyMap(
      cvK_, cvD_, cv::Mat_<double>::eye(3,3), cvK_, cv::Size(width_, height_),
      CV_16SC2, undistort_map1_, undistort_map2_);
}

Vector2d PinholeCamera::cam2img(const Vector3d& xyz) const
{
  Vector2d uv = xyz.head(2) / xyz(2);
  return cam2img(uv);
}

Vector2d PinholeCamera::cam2img(const Vector2d& uv) const
{
  Vector2d px; 
  const double x = uv(0), y = uv(1); 
  if(!distortion_) {
    px(0) = fx_ * x + cx_;
    px(1) = fy_ * y + cy_;
  } else {
    double r2 = x*x + y*y;
    double r4 = r2*r2;
    double r6 = r4*r2;
    double t1 = 2*x*y; 
    double t2 = r2 + 2*x*x; 
    double t3 = r2 + 2*y*y; 
    double xpp = x*(1+r2*k1_+r4*k2_+r6*k3_)+p1_*t1+p2_*t2; 
    double ypp = y*(1+r2*k1_+r4*k2_+r6*k3_)+p1_*t3+p2_*t1; 
    px(0) = fx_ * xpp + cx_; 
    px(1) = fy_ * ypp + cy_; 
  }
  return px;
}

Vector3d PinholeCamera::img2cam(const Vector2d& uv) const
{
  Vector3d xyz;
  const double u = uv(0);
  const double v = uv(1);
  if(!distortion_) {
    xyz(0) = (u - cx_) / fx_;
    xyz(1) = (v - cy_) / fy_;
    xyz(2) = 1.0;
  } else {
    cv::Point2f in(u, v), out;
    const cv::Mat src(1, 1, CV_32FC2, &in.x);
    cv::Mat dst(1, 1, CV_32FC2, &out.x);
    cv::undistortPoints(src, dst, cvK_, cvD_);
    xyz(0) = out.x;
    xyz(1) = out.y;
    xyz(2) = 1.0;
  }
  return xyz.normalized();
}

void PinholeCamera::undistortImg(const cv::Mat& in, cv::Mat& out) const
{
  if(!distortion_) {
    out = in.clone();
  } else {
    cv::remap(in, out, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
  }
}

} // namespace rokit

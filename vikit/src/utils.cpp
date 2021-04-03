#include <opencv2/core.hpp>
#include <vikit/utils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vikit/colors.h>
#include <vikit/derivatives.h>

namespace rokit {
namespace utils {

void drawFeatures(cv::Mat& img, const Features& fts)
{
  for(const auto& ftr: fts)
  {
    Vector3i c = randomColor();
    cv::circle(img,
        cv::Point2f(ftr->px_[0], ftr->px_[1]),
        5.0*(ftr->lvl_+1), cv::Scalar(c(0), c(1), c(2)), -1);
  }
}

void drawFeaturesWithDepth(cv::Mat& img, const Features& fts)
{
  for(const auto& ftr: fts)
  {
    if(ftr->point_ == nullptr)
      continue;
    const Vector3i c = interpolateDepthColor(1.0/(ftr->point_->pt_.z()+1e-7));
    cv::circle(img, cv::Point2f(ftr->px_[0], ftr->px_[1]), 3.0, cv::Scalar(c(0), c(1), c(2)), -1, cv::FILLED);
  }
}

void drawPatches(cv::Mat& img, const Features& fts, const int patch_size)
{
  const int half_patch_size = patch_size/2.0;
  for(const auto& ftr: fts)
  {
    if(ftr->point_ == nullptr)
      continue;
    const Vector3i c = interpolateDepthColor(1.0/(ftr->point_->pt_.z()+1e-7));
    cv::rectangle(img,
        cv::Point2f(ftr->px_[0]-half_patch_size, ftr->px_[1]-half_patch_size),
        cv::Point2f(ftr->px_[0]+half_patch_size, ftr->px_[1]+half_patch_size),
        cv::Scalar(c(0), c(1), c(2)), -1.0);
  }
}

void drawFlow(cv::Mat& img, const Vector2d& px, const Vector2d& flow)
{
  cv::circle(img, cv::Point2f(px(0), px(1)), 2, cv::Scalar(0, 0, 255), -1, cv::FILLED);
  if(flow.norm() < 1e-2)
    return;
  const Vector2d dir = flow.normalized();
  cv::line(img,
      cv::Point2f(px(0), px(1)),
      cv::Point2f(px(0)+dir(0)*20, px(1)+dir(1)*20),
      cv::Scalar(255, 0, 0), 3);
  cv::circle(img, cv::Point2f(px(0), px(1)), 2, cv::Scalar(0, 0, 255), -1, cv::FILLED);
}

void drawFlow(cv::Mat& img, const vector<Vector2d>& px, const vector<Vector2d>& flow)
{
  ROS_ASSERT_MSG(px.size() == flow.size(), "Flow and pixel locations size not equal.");
  for(int i=0; i<px.size(); ++i)
    drawFlow(img, px[i], flow[i]);
}

Vector3i interpolateDepthColor(
    const double inv_depth,
    const double inv_depth_min,
    const double inv_depth_max)
{
  const double alpha = (inv_depth_max - inv_depth) / (inv_depth_max - inv_depth_min);

  // blue -> far and red -> near
  Vector3d i = alpha * colors::blue + (1-alpha) * colors::red;
  Vector3i color(static_cast<int>(i(0)), static_cast<int>(i(1)), static_cast<int>(i(2)));
  return color;
}

void displayImg(
    const std::string window_name,
    const cv::Mat& img,
    const int wait_time_ms)
{
  cv::imshow(window_name.c_str(), img);
  cv::waitKey(wait_time_ms);
}

bool loadImg(
    const string& path,
    cv::Mat& img)
{
  img = cv::imread(path.c_str(), 0);
  if(img.empty())
  {
    ROS_FATAL_STREAM("Invalid image path: " << path);
    return false; 
  }
  if(img.type() != CV_8UC1)
  {
    ROS_FATAL_STREAM("Image type is not 8UC1: " << img.type());
    return false;
  }
  return true;
}

void loadDepthImg(
    const string& path,
    vector<double>& depth)
{
  std::ifstream ifs(path.c_str());
  if(!ifs.good())
  {
    ROS_FATAL_STREAM("Invalid depth data file.");
    return;
  }
  std::string d;
  while(std::getline(ifs, d, ' '))
  {
    const double rho = std::stod(d);
    depth.push_back(rho);
  }
}

void concatImgs(
    const cv::Mat& img_l, const cv::Mat& img_r,
    cv::Mat& img)
{
  ROS_ASSERT_MSG(img_l.rows == img_r.rows, "Invalid input images.");
  ROS_ASSERT_MSG(img_l.type() == img_r.type(), "Invalid input images.");

  img = cv::Mat::zeros(img_l.rows, img_l.cols+img_r.cols, img_l.type());
  img_l.copyTo(img.rowRange(0, img_l.rows).colRange(0, img_l.cols));
  img_r.copyTo(img.rowRange(0, img_l.rows).colRange(img_l.cols, img_l.cols+img_r.cols));
}

Vector3i randomColor()
{
  Vector3i c;
  c(0) = rand() % 255;
  c(1) = rand() % 255;
  c(2) = rand() % 255;
  return c;
}

void constructImgPyramid(
    const cv::Mat& img, ImgPyr& img_pyr)
{
  ROS_ASSERT_MSG(!img.empty(), "Empty image.");

  img_pyr.push_back(img);
  cv::Mat new_img; img.copyTo(new_img);
  for(int i=1; i<=5; ++i)
  {
    cv::pyrDown(new_img, new_img, cv::Size(new_img.cols/2, new_img.rows/2));
    img_pyr.push_back(new_img);
  }
}

double harrisCornerScore(
    const cv::Mat& img, float u, float v)
{
  const Vector2d gradient = imgJac_8uc1(img, u, v);
  const double Ix = gradient(0);
  const double Iy = gradient(1);
  Matrix2d M; M << Ix*Ix, Ix*Iy, Iy*Ix, Iy*Iy;
  return M.determinant() - std::pow(M.trace(), 2);
}

double shiTomasiScore(
    const cv::Mat& img, float u, float v)
{
  // Ref: https://opencv-python-tutroals.readthedocs.io/en/latest/
  //      py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html
  // NOTE: For some reason (noisy?) using only single pixel location
  //       in computing the score, the results weren't good.
  const auto gradient = imgJac_8uc1(img, u, v, 4);
  double Ixx = 0.0;
  double Iyy = 0.0;
  double Ixy = 0.0;
  for(int i=0; i<gradient.rows(); ++i)
  {
    const double Ix = gradient(i, 0);
    const double Iy = gradient(i, 1);
    Ixx += Ix*Ix;
    Iyy += Iy*Iy;
    Ixy += Ix*Iy;
  }
  Ixx /= PATCH_AREA;
  Iyy /= PATCH_AREA;
  Ixy /= PATCH_AREA;

  // min. eigen value
  return 0.5*((Ixx+Iyy) - std::sqrt((Ixx+Iyy)*(Ixx+Iyy) - 4.0*(Ixx*Iyy - Ixy*Ixy)));
}

} // namespace utils
} // namespace rokit

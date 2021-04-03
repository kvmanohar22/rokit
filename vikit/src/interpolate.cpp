#include <cmath>
#include <vikit/interpolate.h>

namespace rokit {
namespace utils {

template <typename T>
T interpolate_8uc1(
    const cv::Mat& img, float u, float v)
{
  ROS_ASSERT_MSG(img.type() == CV_8UC1, "Wrong image type.");

  const int int_u = std::floor(u);
  const int int_v = std::floor(v);
  if(int_u < 1 or int_v < 1 or int_u > img.cols-2 or int_v > img.rows-2)
    return 0;

  const float du = u - int_u;
  const float dv = v - int_v;

  const int stride = img.step.p[0];
  uint8_t* data = img.data + int_v * stride + int_u;

  const float w00 = (1.0f-du) * (1.0f-dv);
  const float w01 = (1.0f-du) * (     dv);
  const float w10 = (     du) * (1.0f-dv);
  const float w11 = (     du) * (     dv);
  T val = w00 * data[0] + w10 * data[1] + w01 * data[stride] + w11 * data[stride+1];
  return val;
}

template float   interpolate_8uc1<float>(const cv::Mat& img, float u, float v);
template uint8_t interpolate_8uc1<uint8_t>(const cv::Mat& img, float u, float v);

float interpolate_32fc1(
    const cv::Mat& img, float u, float v)
{
  ROS_ASSERT_MSG(img.type() == CV_32FC1, "Wrong image type.");

  const int int_u = std::floor(u);
  const int int_v = std::floor(v);
  if(int_u < 1 or int_v < 1 or int_u > img.cols-2 or int_v > img.rows-2)
    return 0;

  const float du = u - int_u;
  const float dv = v - int_v;

  const float w00 = (1.0f-du) * (1.0f-dv);
  const float w01 = (1.0f-du) * (     dv);
  const float w10 = (     du) * (1.0f-dv);
  const float w11 = (     du) * (     dv);

  float val = w00 * img.at<float>(int_v  , int_u  ) +
              w10 * img.at<float>(int_v  , int_u+1) +
              w01 * img.at<float>(int_v+1, int_u  ) +
              w11 * img.at<float>(int_v+1, int_u+1);
  return val;
}

} // namespace utils
} // namespace rokit

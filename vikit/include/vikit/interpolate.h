#ifndef _VIKIT_INTERPOLATE_H_
#define _VIKIT_INTERPOLATE_H_

#include <rk_common/global.h>

namespace rokit {
namespace utils {

/// bilinear interpolation
template <typename T>
T interpolate_8uc1(
    const cv::Mat& img, float u, float v);
extern template float interpolate_8uc1<float>(const cv::Mat& img, float u, float v);
extern template uint8_t interpolate_8uc1<uint8_t>(const cv::Mat& img, float u, float v);

float interpolate_32fc1(
    const cv::Mat& img, float u, float v);

} // namespace utils
} // namespace rokit

#endif // _VIKIT_INTERPOLATE_H_


#ifndef _VIKIT_DERIVATIES_H_
#define _VIKIT_DERIVATIES_H_

#include <rk_common/global.h>

namespace rokit {

/// compute the image gradient at (u, v)
/// uses central finite difference scheme to approximate the derivative
Vector2d imgJac_8uc1(
    const cv::Mat& img,
    const float u,
    const float v);

// consider a patch around it
ImgJacxyType imgJac_8uc1(
    const cv::Mat& img,
    const float u,
    const float v,
    const float half_patch_size);

} // namespace rokit

#endif // _VIKIT_DERIVATIES_H_


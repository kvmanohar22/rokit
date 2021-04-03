#ifndef _VIKIT_UTILS_H_
#define _VIKIT_UTILS_H_

#include <rk_common/global.h>
#include <vikit/feature.h>

namespace rokit {
namespace utils {

/// draw features on image
void drawFeatures(
    cv::Mat& img, const Features& fts);

/// overlay depth of points in blue-red palette
void drawFeaturesWithDepth(
    cv::Mat& img, const Features& fts);

/// draw features on image
void drawPatches(
    cv::Mat& img, const Features& fts, const int patch_size);

/// draw a flow vector
void drawFlow(
    cv::Mat& img,
    const Vector2d& px,
    const Vector2d& flow);

void drawFlow(
    cv::Mat& img,
    const vector<Vector2d>& px,
    const vector<Vector2d>& flow);

/// inv. depth is expressed along the optical z-axis
Vector3i interpolateDepthColor(
    const double inv_depth,
    const double inv_depth_min=0,
    const double inv_depth_max=0.15);

/// display an image
void displayImg(
    const std::string window_name,
    const cv::Mat& img,
    const int wait_time_ms=0);

bool loadImg(
    const string& path,
    cv::Mat& img);

/// loads depth values represented in row-major order
void loadDepthImg(
    const string& path,
    vector<double>& depth);

void concatImgs(
    const cv::Mat& img_l, const cv::Mat& img_r,
    cv::Mat& img);

Vector3i randomColor();

/// downsample an image
void constructImgPyramid(
    const cv::Mat& img, ImgPyr& img_pyr);

double harrisCornerScore(
    const cv::Mat& img, float u, float v);
double shiTomasiScore(
    const cv::Mat& img, float u, float v);


} // namespace utils
} // namespace rokit
#endif // _VIKIT_UTILS_H_


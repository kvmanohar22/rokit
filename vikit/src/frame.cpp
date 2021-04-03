#include <vikit/frame.h>
#include <vikit/utils.h>

namespace rokit {

Frame::Frame(const cv::Mat& img, PinholeCamera* cam)
  : cam_(cam)
{
  utils::constructImgPyramid(img, img_pyr_);
  T_w_f_ = SE3::exp(Vector6d::Zero());
}

Frame::~Frame()
{}

} // namespace rokit

#include <vikit/derivatives.h>
#include <vikit/interpolate.h>

namespace rokit {

Vector2d imgJac_8uc1(
    const cv::Mat& img,
    const float u,
    const float v)
{
  // NOTE: notation is slightly abused.
  //       jac is actually a row vector and NOT column vector
  //       When using this method, transpose this vector
  Vector2d jac; jac.setZero();
  static const float h = 1.0;
  jac(0) = utils::interpolate_8uc1<float>(img, u+h, v) - utils::interpolate_8uc1<float>(img, u-h, v);
  jac(1) = utils::interpolate_8uc1<float>(img, u, v+h) - utils::interpolate_8uc1<float>(img, u, v-h);
  jac /= 2*h;
  return jac;
}

ImgJacxyType imgJac_8uc1(
    const cv::Mat& img,
    const float u,
    const float v,
    const float half_patch_size)
{
  ImgJacxyType jac; jac.setZero();
  const float xmin = u-HALF_PATCH_SIZE;
  const float ymin = v-HALF_PATCH_SIZE;
  const float xmax = u+HALF_PATCH_SIZE;
  const float ymax = v+HALF_PATCH_SIZE;
  if(xmin < 1.0 or ymin < 1.0 or xmax > img.cols-2 or ymax > img.rows-2)
    return jac;

  float yy = ymin, xx = xmin;
  int idx=0;
  for(int y=0; y<PATCH_SIZE; ++y)
  {
    xx = xmin;
    for(int x=0; x<PATCH_SIZE; ++x, ++idx)
    {
      jac.block<1, 2>(idx, 0) = imgJac_8uc1(img, xx, yy).transpose();
      xx += 1.0;
    }
    yy += 1.0;
  }
  return jac;
}

} // namespace rokit

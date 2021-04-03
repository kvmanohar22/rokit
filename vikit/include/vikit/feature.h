#ifndef _VIKIT_FEATURE_H_
#define _VIKIT_FEATURE_H_

#include <rk_common/global.h>
#include <memory>
#include <vikit/point.h>
#include <vikit/frame.h>

namespace rokit {

/// Feature in the image
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FramePtr  frame_;   //!< frame in which this feature was detected
  Vector2d  px_;      //!< pixel location (u/x = px[0] (col), v/y = px[1] (row)) (at level 0)
  Vector3d  b_;       //!< unit bearing vector
  PointPtr  point_;   //!< 3D point corresponding to this feature (in world frame)
  int       lvl_;     //!< level in which this feature was detected

  Feature(FramePtr& frame, const Vector2d& px, int lvl)
    : frame_(frame),
      px_(px),
      b_(frame_->cam_->img2cam(px_)),
      lvl_(lvl)
  {}
};

} // namespace rokit

#endif // _VIKIT_FEATURE_H_


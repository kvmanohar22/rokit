#ifndef _VIKIT_FRAME_H_
#define _VIKIT_FRAME_H_

#include <rk_common/global.h>
#include <rk_common/pinhole_camera.h>

namespace rokit {

/// fwd
class Feature;
typedef std::shared_ptr<Feature> FeaturePtr;
typedef std::list<FeaturePtr> Features;

class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame(const cv::Mat& img, PinholeCamera* camera);
  virtual ~Frame();

  /// position of the frame in the world
  inline Vector3d pos() const { return T_w_f_.translation(); }

  ImgPyr            img_pyr_;    //!< image
  Features          fts_;        //!< features in this frame
  PinholeCamera*    cam_;        //!< camera
  SE3               T_w_f_;      //!< pose of (f)rame in (w)orld
}; // class Frame

} // namespace rokit

#endif // _VIKIT_FRAME_H_


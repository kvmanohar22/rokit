#ifndef _VIKIT_POINT_H_
#define _VIKIT_POINT_H_

#include <rk_common/global.h>
#include <memory>

namespace rokit {

/// fwd
class Frame;
typedef std::shared_ptr<Frame> FramePtr;
typedef std::list<FramePtr> Frames;

class Point
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  Point(const Vector3d& pt, FramePtr& frame);
  virtual ~Point() {}

  /// this point was observed in a new frame
  void addObs(const FramePtr& frame);

  static int  point_counter;
  int         idx_;     //!< unique point idx
  Vector3d    pt_;      //!< 3D point represented in the world frame
  Frames      frames_;  //!< frames in which this feature is observed
}; // class Point

typedef std::shared_ptr<Point> PointPtr;

} // namespace rokit

#endif // _VIKIT_POINT_H_


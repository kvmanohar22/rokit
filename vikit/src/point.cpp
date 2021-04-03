#include <vikit/point.h>

namespace rokit {

int Point::point_counter = 0;

Point::Point(const Vector3d& pt, FramePtr& frame)
  : idx_(++point_counter),
    pt_(pt)
{
  frames_.push_front(frame);
}

void Point::addObs(const FramePtr& frame)
{
  frames_.push_front(frame);
}

} // namespace rokit

#include "rk_common/utils.h"

namespace rokit {
namespace utils {

Matrix3d sqew(const Vector3d& v)
{
  Matrix3d out;
  out <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
  return out;
}

double rad2deg(const double rad)
{
  return rad * (180.0 / PI);
}

double deg2rad(const double deg)
{
  return deg * (PI / 180.0);
}

Vector2d project2d(const Vector3d& v)
{
  return v.head(2) / v(2);
}

} // namespace utils
} // namespace rokit

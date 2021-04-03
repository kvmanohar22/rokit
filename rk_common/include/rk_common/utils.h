#ifndef _RK_COMMON_UTILS_H_
#define _RK_COMMON_UTILS_H_

#include "rk_common/global.h"

namespace rokit {
namespace utils {

/// skew symmetric matrix of a vector
Matrix3d sqew(const Vector3d& v);

/// angle conversions
double rad2deg(const double rad);
double deg2rad(const double deg);

/// projections
Vector2d project2d(const Vector3d& v);

} // namespace utils
} // namespace rokit

#endif // _RK_COMMON_UTILS_H_

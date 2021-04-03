#ifndef _RK_COMMON_PARAMS_HELPER_H_
#define _RK_COMMON_PARAMS_HELPER_H_

#include "rk_common/global.h"

namespace rokit {
namespace utils {

inline bool hasParam(const string& name)
{
  return ros::param::has(name);
}

template <typename T>
T getParam(const string& name)
{
  T value;
  if(hasParam(name))
  {
    ros::param::get(name, value);
    ROS_INFO_STREAM("Found param: " << name << " -> " << value);
    return value;
  } else {
    ROS_ERROR_STREAM("Param: " << name << " not found.");
    return T();
  }
}

template <typename T>
T getParam(const string& name, const T& default_v)
{
  T value;
  if(hasParam(name))
  {
    ros::param::get(name, value);
    ROS_INFO_STREAM("Found param: " << name << " -> " << value);
    return value;
  } else {
    ROS_WARN_STREAM("Param: " << name << " not found. Assigning default value: " << default_v);
    return default_v;
  }
}

} // namespace utils
} // namespace rokit

#endif // _RK_COMMON_PARAMS_HELPER_H_

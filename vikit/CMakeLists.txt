CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(vikit)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
SET(CMAKE_BUILD_TYPE Release)
SET(Ceres_DIR "/usr/local/lib/cmake/Ceres")

FIND_PACKAGE(fast REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  image_transport
  rk_common
)
INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
  ${FAST_INCLUDE_DIRS}
  ${ceres_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS fast Ceres
  CATKIN_DEPENDS roscpp roslib cv_bridge image_transport rk_common
  INCLUDE_DIRS include
  LIBRARIES vikit
)

ADD_LIBRARY(vikit SHARED
  src/frame.cpp
  src/feature_detector.cpp
  src/point.cpp
  src/utils.cpp
  src/derivatives.cpp
  src/interpolate.cpp
  src/image_alignment.cpp
)
LIST(APPEND LINK_LIBS
   ${catkin_LIBRARIES}
   ${fast_LIBRARIES}
   ceres
)
TARGET_LINK_LIBRARIES(vikit ${LINK_LIBS})

ADD_EXECUTABLE(test_feature_detector test/test_feature_detector.cpp)
TARGET_LINK_LIBRARIES(test_feature_detector vikit)

ADD_EXECUTABLE(test_image_align test/test_alignment.cpp)
TARGET_LINK_LIBRARIES(test_image_align vikit)

ADD_EXECUTABLE(test_image_jac test/test_img_jac.cpp)
TARGET_LINK_LIBRARIES(test_image_jac vikit)

ADD_EXECUTABLE(test_depth test/test_depth.cpp)
TARGET_LINK_LIBRARIES(test_depth vikit)

ADD_EXECUTABLE(test_img_pyr test/test_img_pyr.cpp)
TARGET_LINK_LIBRARIES(test_img_pyr vikit)


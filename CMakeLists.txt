cmake_minimum_required(VERSION 3.0.2)
project(zed_open_capture_ros)

## Compile as C++11
add_compile_options(-std=c++11)

cmake_policy(SET CMP0054 NEW)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  std_msgs
)

# add cmake to find hidapi
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

## Find OpenCV
find_package(OpenCV REQUIRED)
find_package(HIDAPI REQUIRED)
# message(STATUS "libhidapi: include dir at ${HIDAPI_INCLUDE_DIRS}")
# message(STATUS "libhidapi: libraries at ${HIDAPI_LIBRARIES}")
# include_directories()
# find_package(zed_open_capture REQUIRED)
# message(STATUS "ZED Open Capture version: ${zed_open_capture_VERSION}")


## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "/usr/local/include/zed-open-capture"
  ${HIDAPI_INCLUDE_DIRS}
)

link_directories(zed_open_capture "/usr/local/lib")

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge std_msgs
)

## Add executable
add_executable(stereo_node src/stereo_node.cpp)
add_executable(stereo_imu_node src/stereo_imu_node.cpp)

set_target_properties(stereo_node PROPERTIES PREFIX "")
set_target_properties(stereo_imu_node PROPERTIES PREFIX "")

target_compile_definitions(stereo_node PUBLIC VIDEO_MOD_AVAILABLE)
target_compile_definitions(stereo_imu_node PUBLIC VIDEO_MOD_AVAILABLE SENSORS_MOD_AVAILABLE)

# Link libraries
target_link_libraries(stereo_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  zed_open_capture
)
target_link_libraries(stereo_imu_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  zed_open_capture
)

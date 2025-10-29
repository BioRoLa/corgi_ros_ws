cmake_minimum_required(VERSION 3.8)

project(corgi_stair LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_walk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)
set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")
set(EIGEN3_INCLUDE_DIR "${LOCAL_PACKAGE_PATH}/include/eigen3")

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_library(${PROJECT_NAME}
  src/stair_climb.cpp
  src/leg_info.cpp
  # src/dbscan.cpp
  src/plane_segmentation.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${CORGI_UTILS}
  ${CORGI_WALK}
  "tf2_geometry_msgs"
  "tf2_eigen"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "visualization_msgs"
  "corgi_msgs"
  "sensor_msgs"
  "pcl_ros"
  "pcl_conversions"
  "tf2_ros"
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs corgi_utils corgi_walk sensor_msgs visualization_msgs tf2_ros tf2_eigen pcl_ros pcl_conversions pcl_msgs)
ament_package()

find_library(CORGI_UTILS corgi_utils)
find_library(CORGI_WALK corgi_walk)

add_subdirectory(src)

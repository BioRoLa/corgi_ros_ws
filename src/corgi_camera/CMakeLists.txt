cmake_minimum_required(VERSION 3.8)

project(corgi_camera LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosconsole REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(X11 REQUIRED)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")  

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR} 
)
ament_export_dependencies(rclcpp rosconsole std_msgs sensor_msgs rosgraph_msgs geometry_msgs corgi_msgs nav_msgs sensor_msgs cv_bridge image_transport pcl_ros pcl_conversions)
ament_package()

add_subdirectory(src)

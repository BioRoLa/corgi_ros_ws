cmake_minimum_required(VERSION 3.8)

project(corgi_odometry LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

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
)
ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs sensor_msgs corgi_utils)
ament_package()

find_library(CORGI_UTILS corgi_utils)

add_subdirectory(src/kinematic)
add_subdirectory(src/KLD_estimation)
add_subdirectory(src/corgi_odometry)
add_subdirectory(tests)

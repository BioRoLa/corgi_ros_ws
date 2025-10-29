cmake_minimum_required(VERSION 3.8)

project(corgi_algo LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_legged REQUIRED)
find_package(corgi_wheeled REQUIRED)
find_package(corgi_hybrid REQUIRED)
find_package(# corgi_transformation
REQUIRED
)
find_package(corgi_gait_selector REQUIRED)

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
  src/algo.cpp
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs joy sensor_msgs corgi_utils corgi_legged corgi_wheeled corgi_hybrid # corgi_transformation
 corgi_gait_selector
)
ament_package()

find_library(CORGI_UTILS corgi_utils)
find_library(CORGI_HYBRID corgi_hybrid)
find_library(CORGI_WHEELED corgi_wheeled)
find_library(CORGI_LEGGED corgi_legged)
find_library(CORGI_GAIT_SELECTOR corgi_gait_selector)
# find_library(CORGI_TRANSFORMATION corgi_transformation)

add_subdirectory(src)

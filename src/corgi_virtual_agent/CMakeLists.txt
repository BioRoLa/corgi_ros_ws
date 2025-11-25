cmake_minimum_required(VERSION 3.8)

project(corgi_virtual_agent LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ProtoBuf REQUIRED)

# Compiler and standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Optional local package path for grpc/core (do not overwrite CMAKE_PREFIX_PATH)
set(LOCAL_PACKAGE_PATH ${CMAKE_PREFIX_PATH} CACHE STRING "Path to grpc and core installation")
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})
set(CMAKE_PREFIX_PATH "/opt/ros/humble;${LOCAL_PACKAGE_PATH}")

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
)
#ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

add_subdirectory(src)

ament_package()
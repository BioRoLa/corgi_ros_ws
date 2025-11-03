cmake_minimum_required(VERSION 3.8)

project(corgi_virtual_agent LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ProtoBuf REQUIRED)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(LOCAL_PACKAGE_PATH "${HOME}/.grpc_local" CACHE STRING "Path to grpc and core installation")
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})
set(CMAKE_PREFIX_PATH "/opt/ros/noetic;${LOCAL_PACKAGE_PATH}")

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
)
ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs)
ament_package()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

# Define the executable and link required libraries
add_executable(corgi_virtual_agent src/corgi_virtual_agent.cpp)

ament_target_dependencies(corgi_virtual_agent
  "rclcpp"
  "std_msgs"
  "rosgraph_msgs"
  "geometry_msgs"
  "Protobuf::protobuf"
)
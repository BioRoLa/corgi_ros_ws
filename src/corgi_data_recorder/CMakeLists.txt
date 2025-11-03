cmake_minimum_required(VERSION 3.8)

project(corgi_data_recorder LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

# Define the executable and link required libraries
add_executable(corgi_data_recorder src/corgi_data_recorder.cpp)

ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)
ament_package()

ament_target_dependencies(corgi_data_recorder
  "rclcpp"
  "std_msgs"
  "rosgraph_msgs"
  "geometry_msgs"
  "corgi_msgs"
  "sensor_msgs"
)

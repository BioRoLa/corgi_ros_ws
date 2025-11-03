cmake_minimum_required(VERSION 3.8)
project(corgi_utils LANGUAGES CXX)   # C not needed

# ----------------------------------------------------------------------
# ROS 2 / ament
# ----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)

# ----------------------------------------------------------------------
# Eigen3 (system or from source)
# ----------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)   # gives Eigen3::Eigen

# ----------------------------------------------------------------------
# Compiler settings (optional but safe)
# ----------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # for clangd / IDEs

# ----------------------------------------------------------------------
# Library (your utils)
# ----------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/leg_model.cpp
  src/bezier.cpp
  src/trajectory_plan.cpp
)

# Eigen include path for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  include
)

# ROS dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rosgraph_msgs
  geometry_msgs
  corgi_msgs
)

# ----------------------------------------------------------------------
# Executable (your node)
# ----------------------------------------------------------------------
add_executable(leg_model
  src/leg_model_main.cpp
)

# Eigen include path for the executable
target_include_directories(leg_model PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  include
)

# ROS dependencies for the executable
ament_target_dependencies(leg_model
  rclcpp
  std_msgs
  rosgraph_msgs
  geometry_msgs
  corgi_msgs
)

# Link your own library (so the node can use leg_model, bezier, …)
target_link_libraries(leg_model ${PROJECT_NAME})

# ----------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME} leg_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs Eigen3
)
ament_package()
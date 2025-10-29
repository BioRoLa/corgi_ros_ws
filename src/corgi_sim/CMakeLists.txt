cmake_minimum_required(VERSION 3.8)

project(corgi_sim LANGUAGES C CXX)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/node_get_field.srv
        srv/node_enable_contact_points_tracking.srv
        srv/get_uint64.srv
        srv/field_get_string.srv
        srv/node_get_contact_points.srv
        srv/set_int.srv
        srv/supervisor_get_from_id.srv
        srv/set_float.srv
        srv/node_get_orientation.srv
        srv/node_get_position.srv
        srv/motor_set_control_pid.srv
        srv/node_get_def.srv
        msg/ContactPoint.msg
        msg/Float64Stamped.msg
    DEPENDENCIES
        geometry_msgs
)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
)
ament_export_dependencies(rclpy rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs message_runtime)
ament_package()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  scripts/webots_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(src)

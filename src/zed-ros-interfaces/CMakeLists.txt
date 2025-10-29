cmake_minimum_required(VERSION 3.5)

project(zed_interfaces)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(#actionlib_msgs
REQUIRED
)
find_package(shape_msgs REQUIRED)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Fix for QtCretor
#list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/")

set(CMAKE_CXX_STANDARD 1 4)

#add_action_files(DIRECTORY action FILES act.action)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/reset_tracking.srv
        srv/reset_roi.srv
        srv/stop_3d_mapping.srv
        srv/toggle_led.srv
        srv/set_pose.srv
        srv/start_svo_recording.srv
        srv/save_3d_map.srv
        srv/start_remote_stream.srv
        srv/stop_svo_recording.srv
        srv/stop_object_detection.srv
        srv/stop_remote_stream.srv
        srv/start_object_detection.srv
        srv/reset_odometry.srv
        srv/set_led_status.srv
        srv/set_roi.srv
        srv/start_3d_mapping.srv
        srv/save_area_memory.srv
        msg/Keypoint3D.msg
        msg/Object.msg
        msg/PosTrackStatus.msg
        msg/Skeleton2D.msg
        msg/ObjectsStamped.msg
        msg/BoundingBox2Df.msg
        msg/BoundingBox2Di.msg
        msg/Keypoint2Df.msg
        msg/BoundingBox3D.msg
        msg/RGBDSensors.msg
        msg/Keypoint2Di.msg
        msg/PlaneStamped.msg
        msg/Skeleton3D.msg
    DEPENDENCIES
        shape_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(message_generation std_msgs geometry_msgs sensor_msgs)
ament_package()

# Install meshes
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

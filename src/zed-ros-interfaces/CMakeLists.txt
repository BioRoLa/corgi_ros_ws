cmake_minimum_required(VERSION 3.5)

project(zed_interfaces)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Fix for QtCretor
#list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/")

set(CMAKE_CXX_STANDARD 1 4)

#add_action_files(DIRECTORY action FILES act.action)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/ResetTracking.srv
        srv/ResetRoi.srv
        srv/Stop3dMapping.srv
        srv/ToggleLed.srv
        srv/SetPose.srv
        srv/StartSvoRecording.srv
        srv/Save3dMap.srv
        srv/StartRemoteStream.srv
        srv/StopSvoRecording.srv
        srv/StopObjectDetection.srv
        srv/StopRemoteStream.srv
        srv/StartObjectDetection.srv
        srv/ResetOdometry.srv
        srv/SetLedStatus.srv
        srv/SetRoi.srv
        srv/Start3dMapping.srv
        srv/SaveAreaMemory.srv
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

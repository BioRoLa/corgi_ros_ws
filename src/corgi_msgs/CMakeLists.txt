cmake_minimum_required(VERSION 3.5)
project(corgi_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/imu.srv
        msg/FsmStateStamped.msg
        msg/PowerCmdStamped.msg
        msg/MotorStateStamped.msg
        msg/PowerStateStamped.msg
        msg/MotorCmdStamped.msg
        msg/ContactStateStamped.msg
        msg/ImpedanceCmd.msg
        msg/FsmCmdStamped.msg
        msg/MotorState.msg
        msg/ImpedanceCmdStamped.msg
        msg/ContactState.msg
        msg/StairPlanes.msg
        msg/WheelCmd.msg
        msg/SteeringStateStamped.msg
        msg/TriggerStamped.msg
        msg/ForceStateStamped.msg
        msg/Headers.msg
        msg/SimDataStamped.msg
        msg/SimContactPoint.msg
        msg/SensorEnableStamped.msg
        msg/ForceState.msg
        msg/MotorCmd.msg
        msg/SteeringCmdStamped.msg
    DEPENDENCIES
        geometry_msgs
)

include_directories(
  
)
ament_export_dependencies(message_generation std_msgs geometry_msgs sensor_msgs)
ament_package()

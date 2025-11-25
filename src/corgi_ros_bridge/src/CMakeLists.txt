# Find required custom libraries
find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
    message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

# Define the executable and link required libraries
add_executable(corgi_ros_bridge corgi_ros_bridge.cpp)
ament_target_dependencies(corgi_ros_bridge rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)

target_include_directories(corgi_ros_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(corgi_ros_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(corgi_ros_bridge
  ${CMAKE_DL_LIBS}
  ${GRPC_PROTO_LIB}
  ${_CORE_LIBRARIES}  # Optional: Only include if this variable is defined
)
  
install(TARGETS corgi_ros_bridge
  DESTINATION lib/${PROJECT_NAME})
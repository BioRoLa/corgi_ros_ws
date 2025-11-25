# Find required custom libraries
find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
    message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

# Define the executable and link required libraries
add_executable(corgi_virtual_agent corgi_virtual_agent.cpp)
ament_target_dependencies(corgi_virtual_agent rclcpp std_msgs rosgraph_msgs geometry_msgs)

target_include_directories(corgi_virtual_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(corgi_virtual_agent PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(corgi_virtual_agent 
    ${CMAKE_DL_LIBS} 
    ${_CORE_LIBRARIES} 
    ${GRPC_PROTO_LIB}
)

install(TARGETS
  corgi_virtual_agent
  DESTINATION lib/${PROJECT_NAME}
)
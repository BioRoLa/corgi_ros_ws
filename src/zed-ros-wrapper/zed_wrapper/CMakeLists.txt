cmake_minimum_required(VERSION 3.5)

project(zed_wrapper)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosconsole REQUIRED)
find_package(nodelet REQUIRED)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###############################################################################
# SOURCES

set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_wrapper_node.cpp)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
)

###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)
###############################################################################

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
)

add_executable(zed_wrapper_node ${NODE_SRC})
ament_target_dependencies(zed_wrapper_node ${LINK_LIBRARIES})

ament_export_dependencies(rclcpp rosconsole zed_nodelets)
ament_package()

###############################################################################
# INSTALL

install(TARGETS
  zed_wrapper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  urdf
  params
  DESTINATION share/${PROJECT_NAME}
)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

###############################################################################
#Add all files in subdirectories of the project in

# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

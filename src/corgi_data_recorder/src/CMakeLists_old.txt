# Define the executable and link required libraries
add_executable(corgi_data_recorder ./corgi_data_recorder.cpp)
target_link_libraries(corgi_data_recorder ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})

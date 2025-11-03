# Define the executable and link required libraries
add_executable(corgi_csv_control ./corgi_csv_control.cpp)
target_link_libraries(corgi_csv_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})

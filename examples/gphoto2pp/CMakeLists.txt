cmake_minimum_required(VERSION 2.8.3)
project(gphoto2pp_example)

add_executable(gphoto2pp_example gphoto2pp_example.cpp)

target_link_libraries(gphoto2pp_example gphoto2 gphoto2pp)

cmake_minimum_required(VERSION 2.8.3)
project(mavlink_example)

include_directories(../../lib/serial/include
                    ../../lib/mavlink)

set(serial_SRCS
    ../../lib/serial/src/serial.cc
    ../../lib/serial/include/serial/serial.h
    ../../lib/serial/include/serial/v8stdint.h
    ../../lib/serial/src/impl/unix.cc
    ../../lib/serial/src/impl/list_ports/list_ports_linux.cc)

## Add serial library
add_library(serial ${serial_SRCS})
if(UNIX AND NOT APPLE)
    target_link_libraries(serial rt)
endif()

## Build your executable
add_executable(mavlink_simple mavlink_simple.cpp)
add_executable(check_digicam check_digicam.cpp)
add_executable(mavlink_lookup mavlink_lookup.cpp)
## Link your executable against the library
target_link_libraries(mavlink_simple serial)
target_link_libraries(check_digicam serial)
target_link_libraries(mavlink_lookup serial)

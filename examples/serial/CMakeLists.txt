cmake_minimum_required(VERSION 2.8.3)
project(serial_example)

include_directories(../../lib/serial/include)

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
add_executable(serial_example serial_example.cc)
## Link your executable against the library
target_link_libraries(serial_example serial)

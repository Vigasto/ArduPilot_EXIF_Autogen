cmake_minimum_required(VERSION 2.8.3)
project(AP_EXIF_Autogen)

find_package (Threads)

include_directories(lib/serial/include
                    lib/mavlink
		    lib/cpp_exiftool/inc)

set(serial_SRCS
    lib/serial/src/serial.cc
    lib/serial/include/serial/serial.h
    lib/serial/include/serial/v8stdint.h
    lib/serial/src/impl/unix.cc
    lib/serial/src/impl/list_ports/list_ports_linux.cc)

set(exiftool_SRCS
    lib/cpp_exiftool/src/ExifTool.cpp
    lib/cpp_exiftool/src/ExifToolPipe.cpp
    lib/cpp_exiftool/src/TagInfo.cpp)

## Add serial library
add_library(exiftool ${exiftool_SRCS})
add_library(serial ${serial_SRCS})
if(UNIX AND NOT APPLE)
    target_link_libraries(serial rt)
endif()

## Build your executable
add_executable(ap_exif_autogen src/main.cpp)
## Link your executable against the library
target_link_libraries(ap_exif_autogen serial gphoto2 gphoto2pp exiftool ${CMAKE_THREAD_LIBS_INIT})

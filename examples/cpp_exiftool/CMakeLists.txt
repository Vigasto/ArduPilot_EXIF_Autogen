cmake_minimum_required(VERSION 2.8.3)
project(serial_example)

include_directories(../../lib/cpp_exiftool/inc)

set(exiftool_SRCS
    ../../lib/cpp_exiftool/src/ExifTool.cpp
    ../../lib/cpp_exiftool/src/ExifToolPipe.cpp
    ../../lib/cpp_exiftool/src/TagInfo.cpp)

## Add serial library
add_library(exiftool ${exiftool_SRCS})

## Build your executable
add_executable(exiftool_example exiftool_example.cpp)
## Link your executable against the library
target_link_libraries(exiftool_example exiftool)

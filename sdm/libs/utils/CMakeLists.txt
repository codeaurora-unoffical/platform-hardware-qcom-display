project(sdmutils)
cmake_minimum_required(VERSION 2.6)

set (LOCAL_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O2")


add_definitions(
    -Wall
    -Wno-missing-field-initializers
    -Wno-endif-labels
    -Wno-missing-braces
    -Wno-unused-parameter
    -std=c++11
    -fPIC
    -fpermissive
)

add_library (sdmutils SHARED
    ${LOCAL_PATH}/debug.cpp
    ${LOCAL_PATH}/rect.cpp
    ${LOCAL_PATH}/sys.cpp
)

include_directories (${LOCAL_PATH})
include_directories (${LOCAL_PATH}/../../include/)
include_directories (${SYSROOTINC_PATH})

link_directories(${SYSROOT_LIBDIR})

target_link_libraries (sdmutils stdc++)
target_link_libraries (sdmutils glib-2.0)
target_link_libraries (sdmutils log)

install (TARGETS sdmutils LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

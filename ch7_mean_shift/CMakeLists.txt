project ()
cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS -Wall -Wextra )
set(CMAKE_CXX_FLAGS  -g)


find_package(OpenCV 3.2.0 REQUIRED)


MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version " )
MESSAGE ( STATUS "libraries " )
MESSAGE ( STATUS "include path " ) 
include_directories()
include_directories(/src)
include_directories(/utils)

add_subdirectory(utils)
add_subdirectory(src)
add_subdirectory(test)

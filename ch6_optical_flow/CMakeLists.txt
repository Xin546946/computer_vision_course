project(optical_flow)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS -Wall -Wextra )
set(CMAKE_CXX_FLAGS  -g)


find_package(OpenCV 3.2.0 REQUIRED)


MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/utils)

add_subdirectory(utils)
add_subdirectory(src)
add_subdirectory(test)

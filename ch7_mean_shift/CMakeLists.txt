cmake_minimum_required(VERSION 3.10.2)
project(mean_shift)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -o3")


find_package(OpenCV 3.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin  REQUIRED)


MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(/usr/include/eigen3)

add_subdirectory(utils)
add_subdirectory(src)
add_subdirectory(test)


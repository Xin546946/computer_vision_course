cmake_minimum_required(VERSION 3.10.2)
project(active_contour)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS  -o3)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")


# gprof
OPTION(USE_GPROF  "Use gprof"  FALSE)

if(USE_GPROF)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
    SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
endif(USE_GPROF)


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


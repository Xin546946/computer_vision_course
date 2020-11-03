cmake_minimum_required(VERSION 3.10.2)
project(active_contour)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-o2")

find_package(OpenCV 3.2.0 REQUIRED)

find_package( Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(snake_model ${CMAKE_SOURCE_DIR}/src/snake_model.cpp)
add_library(display ${CMAKE_SOURCE_DIR}/src/display.cpp)

add_executable(test_snake_model ${CMAKE_SOURCE_DIR}/src/test_snake_model.cpp)
target_link_libraries(test_snake_model ${OpenCV_LIBS} snake_model)


cmake_minimum_required(VERSION 3.10.2)
project(ACTIVE_CONTOUR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS -g)

include_directories( "/usr/include/eigen3" )

find_package(OpenCV 3.2.0 REQUIRED)
MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)


add_executable(test_snake test_snake.cpp)
TARGET_LINK_LIBRARIES (test_snake ${EIGEN_LIBS} ${OpenCV_LIBS})


add_executable(snake_model snake_model.cpp)
TARGET_LINK_LIBRARIES(snake_model ${EIGEN_LIBS} ${OpenCV_LIBS})

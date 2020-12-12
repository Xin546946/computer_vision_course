project(OPTICAL_FLOW)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS -Wall -Wextra)
set(CMAKE_CXX_FLAGS -g)

find_package(OpenCV 3.2.0 REQUIRED)

MESSAGE( STATUS "OpenCV library status: " )
MESSAGE( STATUS "version ${OpenCV_INCLUDE_DIRS}" )
MESSAGE( STATUS "libraries ${OpenCV_libs}" )
MESSAGE( STATUS "include path ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_optical_flow test_optical_flow.cpp)
target_link_libraries(test_optical_flow
${OpenCV_LIBS}
)

add_executable(test_ref test_ref_optical_flow.cpp)
target_link_libraries(test_ref
${OpenCV_LIBS}
)
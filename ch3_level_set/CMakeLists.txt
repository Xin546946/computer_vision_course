cmake_minimum_required(VERSION 3.10.2)
project(active_contour)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-g")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")


find_package(OpenCV 3.2.0 REQUIRED)


MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(gdc_base ${CMAKE_SOURCE_DIR}/src/gradient_descent_base.cpp)
add_library(utils ${CMAKE_SOURCE_DIR}/src/level_set_utils.cpp)
add_library(sdf_map ${CMAKE_SOURCE_DIR}/src/sdf_map.cpp)
add_library(display ${CMAKE_SOURCE_DIR}/src/display.cpp)
add_library(lvlset_lbf ${CMAKE_SOURCE_DIR}/src/level_set_lbf.cpp)
target_link_libraries(lvlset_lbf utils gdc_base sdf_map display,lvlset_cv)

add_library(lvlset_cv ${CMAKE_SOURCE_DIR}/src/level_set_cv.cpp) 
target_link_libraries(lvlset_cv utils gdc_base sdf_map display)


add_executable(test_cv ${CMAKE_SOURCE_DIR}/src/test_cv.cpp)
target_link_libraries(test_cv ${OpenCV_LIBS} utils sdf_map display lvlset_cv gdc_base)
add_executable(test_sdf ${CMAKE_SOURCE_DIR}/src/test_sdf.cpp)
target_link_libraries(test_sdf ${OpenCV_LIBS} utils sdf_map display)
add_executable(test_lbf ${CMAKE_SOURCE_DIR}/src/test_lbf.cpp)
target_link_libraries(test_lbf ${OpenCV_LIBS} utils sdf_map display lvlset_cv lvlset_lbf gdc_base)


cmake_minimum_required(VERSION 3.10.2)
project(kmeans_clustering)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-o2")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(OpenCV 3.2.0 REQUIRED)

find_package( Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(k_means ${CMAKE_SOURCE_DIR}/src/k_means.cpp)

add_executable(test_opencv_kmeans test_opencv_kmeans.cpp)
target_link_libraries(test_opencv_kmeans ${OpenCV_LIBS})

add_executable(test_kmeans test_kmeans.cpp)
target_link_libraries(test_kmeans ${OpenCV_LIBS})

add_executable(test_k_means ${CMAKE_SOURCE_DIR}/src/test_k_means.cpp)
target_link_libraries(test_k_means ${OpenCV_LIBS} k_means)

add_executable(non_negative_matrix_factorization ${CMAKE_SOURCE_DIR}/src/matrix_factorization.cpp)
target_link_libraries(non_negative_matrix_factorization ${EIGEN_LIBS})
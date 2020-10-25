cmake_minimum_required(VERSION 3.10.2)
project(kmeans_clustering)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS -o3)

find_package(OpenCV 3.2.0 REQUIRED)

MESSAGE ( STATUS "OpenCV library status:" )
MESSAGE ( STATUS "version ${OpenCV_VERSION}" )
MESSAGE ( STATUS "libraries ${OpenCV_LIBS}" )
MESSAGE ( STATUS "include path ${OpenCV_INCLUDE_DIRS}" )

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(test_opencv_kmeans test_opencv_kmeans.cpp)
target_link_libraries(test_opencv_kmeans ${EIGEN_LIBS} ${OpenCV_LIBS})

add_executable(test_kmeans test_kmeans.cpp)
target_link_libraries(test_kmeans ${EIGEN_LIBS} ${OpenCV_LIBS})

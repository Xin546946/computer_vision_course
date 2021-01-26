
add_library(vis visualizer.cpp)
target_link_libraries(vis ${OpenCV_LIBS} utils)


add_library(hough hough.cpp)
target_link_libraries(hough ${OpenCV_LIBS} utils vis)


add_library(ransac ransac.cpp)
target_link_libraries(ransac ${OpenCV_LIBS} utils)
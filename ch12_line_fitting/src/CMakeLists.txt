add_library(hough hough.cpp)
target_link_libraries(hough ${OpenCV_LIBS} utils)


add_library(ransac ransac.cpp)
target_link_libraries(ransac ${OpenCV_LIBS} utils)
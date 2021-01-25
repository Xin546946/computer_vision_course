add_library(hough hough.cpp)
target_link_libraries(hough ${OpenCV_LIBS} utils)

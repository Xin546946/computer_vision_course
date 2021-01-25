add_executable(test_hough test_hough.cpp)
target_link_libraries(test_hough hough ${OpenCV_LIBS} utils)

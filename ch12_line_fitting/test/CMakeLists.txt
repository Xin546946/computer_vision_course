add_executable(test_hough test_hough.cpp)
target_link_libraries(test_hough hough ${OpenCV_LIBS} utils)

add_executable(test_ransac test_ransac.cpp)
target_link_libraries(test_ransac ransac vis ${OpenCV_LIBS} utils)
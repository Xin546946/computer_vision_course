add_executable(test_hough test_hough.cpp)
target_link_libraries(test_hough hough ${OpenCV_LIBS} utils)

add_executable(test_ransac test_ransac.cpp)
target_link_libraries(test_ransac vis ransac ${OpenCV_LIBS} utils)

add_executable(test_vis test_visualizer.cpp)
target_link_libraries(test_vis vis ${OpenCV_LIBS} utils)

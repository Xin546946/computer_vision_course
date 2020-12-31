
add_executable(test_ms_tracking test_ms_tracking.cpp)
target_link_libraries(test_ms_tracking  ${OpenCV_LIBS}  utils ms_tracking ms_tracker)

add_executable(test_ms_segmentation test_ms_segmentation.cpp)
target_link_libraries(test_ms_segmentation ms_segmentation visualizer_3d)

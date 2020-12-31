add_executable(test_gauss test_gauss_data_generator.cpp)
target_link_libraries(test_gauss utils)

add_executable(test_mean_shift test_mean_shift.cpp)
target_link_libraries(test_mean_shift data_base mean_shift visualizer utils)

add_executable(test_bst test_bst.cpp)
target_link_libraries(test_bst bst utils)

add_executable(test_kdtree test_kdtree.cpp)
target_link_libraries(test_kdtree kdtree utils)

add_executable(test_gkdtree test_gkdtree.cpp)
target_link_libraries(test_gkdtree gkdtree utils)

add_executable(test_gkdtree_mean_shift test_gkdtree_mean_shift.cpp)
target_link_libraries(test_gkdtree_mean_shift data_base mean_shift visualizer utils)


add_executable(test_mean_shift_tracker test_mean_shift_tracker.cpp)
target_link_libraries(test_mean_shift_tracker ${OpenCV_LIBS}  utils mean_shift_tracker  mean_shift data_base)


add_executable(test_single_func test_single_func.cpp)
target_link_libraries(test_single_func ${OpenCV_LIBS}  utils mean_shift_tracker  mean_shift data_base)

add_executable(test_ms_tracking test_ms_tracking.cpp)
target_link_libraries(test_ms_tracking  ${OpenCV_LIBS}  utils ms_tracking ms_tracker)

add_executable(test_class_histogram test_class_histogram.cpp)
target_link_libraries(test_class_histogram ${OpenCV_LIBS} utils)

add_executable(test_ms_segmentation test_ms_segmentation.cpp)
target_link_libraries(test_ms_segmentation ms_segmentation visualizer_3d)

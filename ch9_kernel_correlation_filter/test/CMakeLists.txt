# add_executable(test_cf_tracker test_cf_tracker.cpp)
# target_link_libraries(test_cf_tracker cf_tracker utils ${OpenCV_LIBS})

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft utils ${OpenCV_LIBS})

add_executable(test_bbox test_bbox)
target_link_libraries(test_bbox utils)

add_executable(test_random_affine_transformation test_rand_affine_transformation.cpp)
target_link_libraries(test_random_affine_transformation utils ${OpenCV_LIBS})
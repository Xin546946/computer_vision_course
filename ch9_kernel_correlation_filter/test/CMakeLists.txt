# add_executable(test_cf_tracker test_cf_tracker.cpp)
# target_link_libraries(test_cf_tracker cf_tracker utils ${OpenCV_LIBS})

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft utils ${OpenCV_LIBS})
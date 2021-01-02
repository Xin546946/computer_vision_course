add_executable(test_pf_tracker test_pf_tracker.cpp)
target_link_libraries(test_pf_tracker ${OpenCV_LIBS} pf_tracker)

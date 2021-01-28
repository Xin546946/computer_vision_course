add_executable(test_pf_tracker test_pf_tracker.cpp)
target_link_libraries(test_pf_tracker ${OpenCV_LIBS} pf_tracker)

add_executable(test_pi test_pi.cpp)
target_link_libraries(test_pi ${OpenCV_LIBS} utils)

add_executable(test_moment_estimation test_moment_estimation)
target_link_libraries(test_moment_estimation utils)
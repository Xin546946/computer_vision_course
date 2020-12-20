add_executable(test_gauss test_gauss_data_generator.cpp)
target_link_libraries(test_gauss utils)

add_executable(test_mean_shift test_mean_shift.cpp)
target_link_libraries(test_mean_shift data_base mean_shift)




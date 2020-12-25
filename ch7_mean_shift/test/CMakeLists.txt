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

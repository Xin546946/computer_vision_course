add_executable(test_incremental_gmm test_incremental_gmm.cpp)
target_link_libraries(test_incremental_gmm 
    gmm
)

add_executable(test_motion_based_segmentaion test_motion_based_segmentaion.cpp)
target_link_libraries(test_motion_based_segmentaion 
    mb_seg
)

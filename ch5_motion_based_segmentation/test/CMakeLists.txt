add_executable(test_incremental_gmm test_incremental_gmm.cpp)
target_link_libraries(test_incremental_gmm 
    gmm
)

add_executable(test_motion_seg test_motion_seg.cpp)
target_link_libraries(test_motion_seg 
    gmm
    motion_seg
    ${OpenCV_LIBS}
    utils
)

add_executable(test_motion_seg test_motion_seg.cpp)
target_link_libraries(test_motion_seg 
    gmm
    motion_seg
    ${OpenCV_LIBS}
    utils
)

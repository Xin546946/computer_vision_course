
add_library(gmm gmm.cpp)

add_library(motion_seg motion_seg.cpp)
target_link_libraries(motion_seg ${OpenCV_LIBS})



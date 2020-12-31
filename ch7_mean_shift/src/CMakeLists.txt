
add_library(visualizer_3d visualizer_3d.cpp)
target_link_libraries(visualizer_3d ${OpenCV_LIBS})


add_library(ms_segmentation ms_segmentation.cpp)
target_link_libraries(ms_segmentation ${OpenCV_LIBS} utils visualizer_3d)

add_library(ms_tracking ms_tracking.cpp)
target_link_libraries(ms_tracking ${OpenCV_LIBS} utils)

add_library(ms_tracker ms_tracker.cpp)
target_link_libraries(ms_tracker ms_tracking ${OpenCV_LIBS} utils)
add_library(data_base data_base.cpp)
target_link_libraries(data_base kdtree ${OpenCV_LIBS})

add_library(mean_shift mean_shift.cpp)

add_library(visualizer visualizer.cpp)
target_link_libraries(visualizer ${Pangolin_LIBRARIES})

add_library(visualizer_3d visualizer_3d.cpp)
target_link_libraries(visualizer_3d ${OpenCV_LIBS})

add_library(bst bst.cpp)

add_library(kdtree kdtree.cpp)

add_library(gkdtree gkdtree.cpp)

add_library(tracking_data_base tracking_data_base.cpp)
target_link_libraries(tracking_data_base utils data_base ${Pangolin_LIBRARIES})

add_library(mean_shift_tracker mean_shift_tracker.cpp)
target_link_libraries(mean_shift_tracker tracking_data_base)

add_library(ms_segmentation mean_shift_segmentation.cpp)
target_link_libraries(ms_segmentation ${OpenCV_LIBS} utils visualizer_3d)

add_library(ms_tracking mean_shift_tracking.cpp)
target_link_libraries(ms_tracking ${OpenCV_LIBS} utils)

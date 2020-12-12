add_library(feature_points_manager feature_points_manager.cpp)
target_link_libraries(feature_points_manager utils ${OpenCV_LIBS})

add_library(optical_flow_tracker optical_flow_tracker.cpp)
target_link_libraries(optical_flow_tracker ${OpenCV_LIBS})

# add_library(bounding_box bounding_box.h)
# target_link_libraries(bounding_box ${OpenCV_LIBS})
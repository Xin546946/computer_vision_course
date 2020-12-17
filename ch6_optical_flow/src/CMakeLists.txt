add_library(feature_points_manager feature_points_manager.cpp)
target_link_libraries(feature_points_manager utils ${OpenCV_LIBS})

add_library(optical_flow_tracker optical_flow_tracker.cpp)
target_link_libraries(optical_flow_tracker feature_points_manager ${OpenCV_LIBS})

add_library(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow utils ${OpenCV_LIBS})
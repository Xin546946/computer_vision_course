add_executable(test_record_dataset test_record_dataset.cpp)
target_link_libraries(test_record_dataset 
${OpenCV_LIBS}
utils
)

add_executable(test_optical_flow_tracker test_optical_flow_tracker.cpp)
target_link_libraries(test_optical_flow_tracker 
feature_points_manager
optical_flow_tracker
${OpenCV_LIBS}
utils
)


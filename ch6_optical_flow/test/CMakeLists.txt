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

add_executable(test_of_opencv test_optical_flow_tracker_opencv.cpp)
target_link_libraries(test_of_opencv
${OpenCV_LIBS}
)

add_executable(test_put_val test_put_val.cpp)
target_link_libraries(test_put_val
${OpenCV_LIBS}
utils
)

add_executable(test_status test_status.cpp)
target_link_libraries(test_status
${OpenCV_LIBS}
utils
feature_points_manager
)

add_executable(test_optical_flow test_optical_flow.cpp)
target_link_libraries(test_optical_flow
${OpenCV_LIBS}
utils
optical_flow
)

add_executable(test_ncc test_ncc.cpp)
target_link_libraries(test_ncc
${OpenCV_LIBS}
)


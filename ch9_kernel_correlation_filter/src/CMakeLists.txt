add_library(cf_tracker cf_tracker.cpp)
target_link_libraries(cf_tracker ${OpenCV_LIBS} utils)

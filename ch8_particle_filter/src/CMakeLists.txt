add_library(pf particle_filter.cpp)
target_link_libraries(pf ${OpenCV_LIBS} utils)

add_library(pf_tracker pf_tracker.cpp)
target_link_libraries(pf_tracker pf ${OpenCV_LIBS} utils)

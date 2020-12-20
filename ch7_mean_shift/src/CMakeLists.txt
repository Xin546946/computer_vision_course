add_library(data_base data_base.cpp)
target_link_libraries(data_base ${OpenCV_LIBS})

add_library(mean_shift mean_shift.cpp)

add_library(visualizer visualizer.cpp)
target_link_libraries(visualizer ${Pangolin_LIBRARIES})


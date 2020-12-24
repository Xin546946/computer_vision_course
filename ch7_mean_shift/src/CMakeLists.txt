add_library(data_base data_base.cpp)
target_link_libraries(data_base kdtree ${OpenCV_LIBS})

add_library(mean_shift mean_shift.cpp)

add_library(visualizer visualizer.cpp)
target_link_libraries(visualizer ${Pangolin_LIBRARIES})

add_library(bst bst.cpp)

add_library(kdtree kdtree.cpp)

add_library(gkdtree gkdtree.cpp)
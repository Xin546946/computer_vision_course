add_library(graph_search graph_search.cpp)
target_link_libraries(graph_search ${OpenCV_LIBS})

add_library(ek ek.cpp)
target_link_libraries(ek graph_search)

add_library(graph graph.cpp)
# target_link_libraries(graph graph_search)
add_library(image_graph image_graph.cpp)
target_link_libraries(image_graph graph ${OpenCV_LIBS})

add_library(graph_search graph_search.cpp)

add_library(ek ek.cpp)
target_link_libraries(ek graph_search)


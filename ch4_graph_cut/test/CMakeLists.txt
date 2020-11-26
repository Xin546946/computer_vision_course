add_executable(test_ek test_ek.cpp)
target_link_libraries(test_ek 
    ek
)


add_executable(test_dfs test_dfs.cpp)
target_link_libraries(test_dfs 
    graph_search
    ek
)

add_executable(test_graph test_graph.cpp)
target_link_libraries(test_graph
    graph_search
    ek
    graph
)

add_executable(test_image_graph test_image_graph.cpp)
target_link_libraries(test_image_graph
    graph_search
    ek
    graph
    image_graph
)

add_executable(test_distribution test_distribution.cpp)
target_link_libraries(test_distribution
    utils
    ${OpenCV_LIBS}
    distribution
    interaction_tool
)

add_executable(test_gmm test_gmm.cpp)
target_link_libraries(test_gmm
    utils
    ${OpenCV_LIBS}
    gmm
    em
    #distribution
)





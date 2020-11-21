add_executable(test_ek test_ek.cpp)
target_link_libraries(test_ek 
    ek
)


add_executable(test_dfs test_dfs.cpp)
target_link_libraries(test_dfs 
    graph_search
    ek
)


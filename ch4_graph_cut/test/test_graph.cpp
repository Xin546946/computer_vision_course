#include "graph.h"
#include <iostream>

int main(int argc, char** argv) {
    int num_nodes;
    Graph graph(num_nodes);

    graph.add_binary_edge(2, 3, 0.5);
    graph.add_binary_edge(2, 1e5, 0.5);
    graph.add_unary_edge(0, 5, 0.8);
    graph.add_unary_edge(0, 7, 0.8);

    DFS(graph.get_root());

    return 0;
}
#include "graph.h"
#include <iostream>

int main(int argc, char** argv) {
    int num_node;
    Graph graph(num_node);

    graph.add_binary_node(2, 3, 0.5);
    graph.add_binary_node(2, 1e5, 0.5);
    graph.add_unary_node(0, 5, 0.8);
    graph.add_unary_node(0, 7, 0.8);

    DFS(graph.get_root());

    return 0;
}
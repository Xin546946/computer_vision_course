#include "graph.h"
#include <iostream>

class WeightedEdge : EdgeBase {
   public:
    WeightedEdge() = default;
    WeightedEdge(double weight) : EdgeBase(), weight_(weight){};
    double weight_;
};

int main(int argc, char** argv) {
    int num_node = 10;
    Graph<NodeBase<WeightedEdge>, WeightedEdge> graph(num_node);

    graph.add_binary_edge(2, 3, WeightedEdge(0.5));
    graph.add_binary_edge(2, 1e5, WeightedEdge(0.5));
    graph.add_unary_edge(0, 5, WeightedEdge(0.5));
    graph.add_unary_edge(0, 7, WeightedEdge(0.8));

    // DFS(graph.get_root());

    return 0;
}

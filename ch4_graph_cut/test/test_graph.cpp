#include "graph.h"
#include "graph_search.h"
#include <iostream>

class WeightedEdge : EdgeBase {
   public:
    WeightedEdge() = default;
    WeightedEdge(double weight) : EdgeBase(), weight_(weight){};
    double weight_;
};

int main(int argc, char** argv) {
    int n = 7;
    Graph<NodeBase<WeightedEdge>, WeightedEdge> graph(n);
    graph.add_unary_edge(0, 1, new WeightedEdge(1.3));
    graph.add_unary_edge(0, 2, new WeightedEdge(1.4));
    graph.add_unary_edge(1, 3, new WeightedEdge(1.5));
    graph.add_unary_edge(1, 4, new WeightedEdge(1.5));
    graph.add_unary_edge(2, 4, new WeightedEdge(1.5));
    graph.add_unary_edge(2, 5, new WeightedEdge(1.5));
    graph.add_unary_edge(3, 6, new WeightedEdge(1.5));
    graph.add_unary_edge(4, 6, new WeightedEdge(1.5));

    std::unordered_set<NodeBase<WeightedEdge>*> visited;
    DFS(graph.get_root(), visited);

    return 0;
}

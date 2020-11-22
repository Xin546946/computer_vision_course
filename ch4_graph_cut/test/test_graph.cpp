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
    int n = 11;
    int s = n - 2;
    int t = n - 1;
    Graph<NodeBase<WeightedEdge>, WeightedEdge> graph(n);

    graph.add_unary_edge(s, 0, WeightedEdge(5));
    graph.add_unary_edge(s, 1, WeightedEdge(10));
    graph.add_unary_edge(s, 2, WeightedEdge(5));

    graph.add_unary_edge(0, 3, WeightedEdge(10));
    graph.add_unary_edge(1, 0, WeightedEdge(15));
    graph.add_unary_edge(1, 4, WeightedEdge(20));
    graph.add_unary_edge(2, 5, WeightedEdge(10));
    graph.add_unary_edge(3, 4, WeightedEdge(25));
    graph.add_unary_edge(3, 6, WeightedEdge(10));
    graph.add_unary_edge(4, 2, WeightedEdge(5));
    graph.add_unary_edge(4, 7, WeightedEdge(30));
    graph.add_unary_edge(5, 7, WeightedEdge(5));
    graph.add_unary_edge(5, 8, WeightedEdge(10));
    graph.add_unary_edge(7, 3, WeightedEdge(15));
    graph.add_unary_edge(7, 8, WeightedEdge(5));

    graph.add_unary_edge(6, t, WeightedEdge(5));
    graph.add_unary_edge(7, t, WeightedEdge(15));
    graph.add_unary_edge(8, t, WeightedEdge(25));

    DFS(graph.get_root());

    return 0;
}

#pragma once
#include <vector>

template <typename TypeEdge>
struct Node {
    Node(int id);
    std::vector<std::pair<Node*, TypeEdge>> neighbour_;
    // std::pair<Node*, Edge*> parent_;
    void g int id_;
};

struct Edge {
    Edge(int cap);
    int cap_;
    int flow_;
    int get_residual();
    bool is_full();
};

class Graph {
   public:
    Graph(int num_nodes);
    void add_binary_edge(int node1, int node2, double edge_weight);
    void add_unary_edge(int src_node, int target_node, double edge_weight);
    void get_root();

   private:
    int num_nodes_;
};

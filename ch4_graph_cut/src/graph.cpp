#include "graph.h"

Graph::Graph(int num_nodes) : num_nodes_(num_nodes) {
}

void Graph::add_binary_edge(int node1, int node2, double edge_weight) {
    node1.add_neighbour(node2, edge_weight);
    node2.add_neighbour(node1, edge_weight);
}

void Graph::add_unary_edge(int src_node, int target_node, double edge_weight) {
    src_node.add_neighbour(target_node, edge_weight);
}

void Node::add_neighbour(target_node, edge_weight) {
}

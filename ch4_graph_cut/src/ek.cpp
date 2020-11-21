#include "ek.h"

Node::Node(int id) : id_(id) {
}

EKSolver::EKSolver(int num_nodes, int src_id, int sink_id)
    : src_id_(src_id), sink_id_(sink_id) {
    for (int id = 0; id < num_nodes; id++) {
        nodes_.emplace_back(id);  // put id to the ctor of nodes[id]
    }
}

void EKSolver::add_edge(int parent_id, int child_id, int edge_weight) {
    nodes_[parent_id].children_.emplace_back(&nodes_[child_id], edge_weight);
}

// int get_max_flow(){}

Node* EKSolver::get_graph() {
    return &nodes_[src_id_];
}
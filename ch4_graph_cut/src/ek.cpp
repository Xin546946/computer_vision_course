#include "ek.h"

Edge::Edge(int cap) : cap_(cap), flow_(0) {
}

int Edge::get_residual() {
    return cap_ - flow_;
}

Node::Node(int id) : id_(id) {
}

EKSolver::EKSolver(int num_nodes, int src_id, int sink_id)
    : src_id_(src_id), sink_id_(sink_id) {
    for (int id = 0; id < num_nodes; id++) {
        nodes_.emplace_back(id);  // put id to the ctor of nodes[id]
    }
}

void EKSolver::add_edge(int parent_id, int child_id, int cap) {
    nodes_[parent_id].children_.emplace_back(&nodes_[child_id], Edge(cap));
}

// int get_max_flow(){}

Node* EKSolver::get_graph() {
    return &nodes_[src_id_];
}
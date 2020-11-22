#include "ek.h"
#include "graph_search.h"
#include <iostream>
#include <limits>
EdgeEK::EdgeEK(int cap) : cap_(cap), flow_(0) {
}

int EdgeEK::get_residual() {
    return cap_ - flow_;
}

bool EdgeEK::is_full() {
    return get_residual() <= 0;
}

NodeEK::NodeEK(int id) : id_(id) {
}
EKSolver::EKSolver(int num_nodes, int src_id, int sink_id)
    : src_id_(src_id), sink_id_(sink_id) {
    for (int id = 0; id < num_nodes; id++) {
        nodes_.emplace_back(id);  // put id to the ctor of nodes[id]
    }
}

void EKSolver::add_edge(int parent_id, int child_id, int cap) {
    std::cout << "cap :" << cap << '\n';
    nodes_[parent_id].children_.emplace_back(&nodes_[child_id], EdgeEK(cap));
}

int EKSolver::compute_max_flow() {
    while (true) {
        // step 1 : do bfs
        std::vector<std::pair<NodeEK*, EdgeEK*>> path =
            BFS_get_path(get_graph(), sink_id_, src_id_);
        // step2 :check terminnation
        if (path.empty() || path.front().first->id_ != sink_id_) {
            break;
        }
        // step3 :bottenlenck
        int min_residual = std::numeric_limits<int>::max();
        for (int i = 0; i < path.size(); i++) {
            min_residual =
                std::min(min_residual, path[i].second->get_residual());
        }
        // step4 :update graph
        for (int i = 0; i < path.size(); i++) {
            std::cout << "min_residual :" << min_residual << std::endl;
            std::cout << "flow before :" << path[i].second->flow_ << '\n';
            path[i].second->flow_ += min_residual;
            std::cout << "flow after :" << path[i].second->flow_ << '\n';
        }
    }

    return BFS(get_graph(), sink_id_);
}

NodeEK* EKSolver::get_graph() {
    return &nodes_[src_id_];
}
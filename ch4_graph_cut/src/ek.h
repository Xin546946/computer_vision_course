
#pragma once
#include <vector>

struct EdgeEK {
    EdgeEK(int cap);
    int cap_;
    int flow_;
    int get_residual();
    bool is_full();
};
/**
 * @brief Create a Node, whose children are corresponding nodes and weight
 *
 */
struct NodeEK {
    NodeEK(int id);
    std::vector<std::pair<NodeEK*, EdgeEK>> children_;
    std::pair<NodeEK*, EdgeEK*> parent_;
    int id_;
};

class EKSolver {
   public:
    EKSolver(int num_nodes, int src_id, int sink_id);
    void add_edge(int parent_id, int child_id, int edge_weight);
    int compute_max_flow();
    NodeEK* get_graph();

   private:
    int src_id_;
    int sink_id_;
    std::vector<NodeEK> nodes_;
    // Node src;
    // Node sink;
};

#pragma once
#include <vector>
/**
 * @brief Create a Node, whose children are corresponding nodes and weight
 *
 */
struct Node {
    Node(int id);
    std::vector<std::pair<Node*, int>> children_;
    int id_;
};

class EKSolver {
   public:
    EKSolver(int num_nodes, int src_id, int sink_id);
    void add_edge(int parent_id, int child_id, int edge_weight);
    int get_max_flow() const;

   private:
    int src_id_;
    int sink_id_;
    std::vector<Node> nodes_;
    // Node src;
    // Node sink;
};

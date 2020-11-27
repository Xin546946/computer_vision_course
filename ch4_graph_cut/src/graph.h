#pragma once
#include <iostream>
#include <vector>

template <typename TypeEdge>
struct NodeBase {
    NodeBase(int id);
    int id_;
    std::vector<std::pair<NodeBase<TypeEdge>*, TypeEdge>> neighbours_;
    void add_neighbour(NodeBase<TypeEdge>* target_node, const TypeEdge& edge);

    void back_up_prev(NodeBase<TypeEdge>*, TypeEdge*);
    std::pair<NodeBase<TypeEdge>*, TypeEdge*> prev_;

    //    protected:
    //     std::pair<NodeBase<TypeEdge>*, TypeEdge*> get_prev() const;

    // std::vector<std::pair<NodeBase<TypeEdge>*, TypeEdge>> get_neighbours()
    // const;
};

class EdgeBase {
   public:
    EdgeBase() = default;

   protected:
};

template <typename TypeNode, typename TypeEdge>
class Graph {
   public:
    Graph(int num_nodes);
    void add_binary_edge(int id_node1, int id_node2,
                         const TypeEdge& edge_weight);
    void add_unary_edge(int id_src, int id_target, const TypeEdge& edge_weight);
    TypeNode* get_root();

   protected:
    std::vector<TypeNode> nodes_;
};

/*--------------------------------------------------------
#####################implementation: Graph #####################
---------------------------------------------------------*/

template <typename TypeNode, typename TypeEdge>
Graph<TypeNode, TypeEdge>::Graph(int num_nodes) {
    for (int i = 0; i < num_nodes; i++) {
        nodes_.emplace_back(i);
    }
}

template <typename TypeNode, typename TypeEdge>
void Graph<TypeNode, TypeEdge>::add_binary_edge(int id_node1, int id_node2,
                                                const TypeEdge& edge) {
    if (id_node1 < nodes_.size() && id_node2 < nodes_.size()) {
        nodes_[id_node1].add_neighbour(&nodes_[id_node2], edge);
        nodes_[id_node2].add_neighbour(&nodes_[id_node1], edge);
    } else {
        std::cout << "node id is over maximum ! addition will be ignored."
                  << std::endl;
    }
}

template <typename TypeNode, typename TypeEdge>
void Graph<TypeNode, TypeEdge>::add_unary_edge(int id_src, int id_target,
                                               const TypeEdge& edge) {
    nodes_[id_src].add_neighbour(&nodes_[id_target], edge);
}

template <typename TypeNode, typename TypeEdge>
TypeNode* Graph<TypeNode, TypeEdge>::get_root() {
    return &nodes_[0];
}

/*--------------------------------------------------------
#####################implementation: NodeBase #####################
---------------------------------------------------------*/
template <typename TypeEdge>
NodeBase<TypeEdge>::NodeBase(int id) : id_(id), prev_{nullptr, nullptr} {
}

// template <typename TypeEdge>
// int NodeBase<TypeEdge>::id_ const {
//     return id_;
// }

template <typename TypeEdge>
void NodeBase<TypeEdge>::add_neighbour(NodeBase<TypeEdge>* target_node,
                                       const TypeEdge& edge) {
    neighbours_.emplace_back(target_node, edge);
}

// template <typename TypeEdge>
// std::vector<std::pair<NodeBase<TypeEdge>*, TypeEdge>>
// NodeBase<TypeEdge>::get_neighbours() const {
//     return neighbours_;
// }

// template <typename TypeEdge>
// std::pair<NodeBase<TypeEdge>*, TypeEdge*> NodeBase<TypeEdge>::get_prev()
// const {
//     return prev_;
// }

template <typename TypeEdge>
void NodeBase<TypeEdge>::back_up_prev(NodeBase<TypeEdge>* prev_node,
                                      TypeEdge* edge) {
    prev_.first = prev_node;
    prev_.second = edge;
}
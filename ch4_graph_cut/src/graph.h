#pragma once
#include <iostream>
#include <vector>

template <typename TypeEdge>
class NodeBase {
   public:
    NodeBase() = default;
    NodeBase(int id);
    void add_neighbour(NodeBase* target_node, const TypeEdge& edge);

   private:
    std::vector<std::pair<NodeBase*, TypeEdge>> neighbour_;
    int id_;
};

class EdgeBase {
   public:
    EdgeBase() = default;

   private:
};

template <typename TypeNode, typename TypeEdge>
class Graph {
   public:
    Graph(int num_nodes);
    void add_binary_edge(int id_node1, int id_node2,
                         const TypeEdge& edge_weight);
    void add_unary_edge(int id_src, int id_target, const TypeEdge& edge_weight);
    void get_root();

   private:
    std::vector<TypeNode> nodes_;
};

template <typename TypeEdge>
NodeBase<TypeEdge>::NodeBase(int id) : id_(id) {
}

template <typename TypeNode, typename TypeEdge>
Graph<TypeNode, TypeEdge>::Graph(int num_nodes)
    : nodes_(std::vector<TypeNode>(num_nodes)) {
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

template <typename TypeEdge>
void NodeBase<TypeEdge>::add_neighbour(NodeBase* target_node,
                                       const TypeEdge& edge) {
    neighbour_.emplace_back(target_node, edge);
}
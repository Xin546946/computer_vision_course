/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once
#include <vector>
/**
 * @brief Create an Edge, which is aiming to solve network problems
 *
 */
struct EdgeEK {
    /**
     * @brief Construct a new Edge E K object
     *
     * @param cap
     */
    EdgeEK(int cap);
    int cap_;   // cap_ is GMM from Distribution class
    int flow_;  // flow_ is the flow in the edge
    /**
     * @brief Get residual of the edge, cap_ - flow_
     *
     * @return int
     */
    int get_residual();
    /**
     * @brief Check if the edge is full, which means the residual is not zeros
     *
     * @return true
     * @return false
     */
    bool is_full();
};
/**
 * @brief Create a Node, whose children are corresponding nodes and weight
 *
 */
struct NodeEK {
    /**
     * @brief Construct a new Node E K object
     *
     * @param id
     */
    NodeEK(int id);
    std::vector<std::pair<NodeEK*, EdgeEK>> children_;
    std::pair<NodeEK*, EdgeEK*> parent_;
    int id_;
};
/**
 * @brief Edmonds Karp Method for solving Max flow problem
 *
 */
class EKSolver {
   public:
    EKSolver(int num_nodes, int src_id, int sink_id);
    /**
     * @brief Create an edge from parent id, child id and edge weight
     *
     * @param parent_id
     * @param child_id
     * @param edge_weight
     */
    void add_edge(int parent_id, int child_id, int edge_weight);
    /**
     * @brief Compute max flow method
     *
     * @return int
     */
    int compute_max_flow();
    /**
     * @brief Get the graph object
     *
     * @return NodeEK*
     */
    NodeEK* get_root();

   private:
    int src_id_;
    int sink_id_;
    std::vector<NodeEK> nodes_;
    // Node src;
    // Node sink;
};

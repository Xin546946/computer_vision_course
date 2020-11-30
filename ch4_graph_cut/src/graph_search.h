/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once
#include "ek.h"
#include "graph.h"
#include "image_graph.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <queue>
#include <unordered_set>

/**
 * @brief basic deepeth first search for the ek solver
 *
 * @param root: start node
 * @param visited : true if the node has been visited
 */
void DFS(NodeEK* root, std::vector<bool>& visited);

/**
 * @brief basic breadth first search for the ek solver
 *
 * @param root start node
 */
void BFS(NodeEK* root);

/**
 * @brief return all the flow to a specific node with given id for ek solver
 *
 * @param root: start node
 * @param id_target: id of the target node
 * @return int : flow to a specific node
 */
int BFS(NodeEK* root, int id_target);

/**
 * @brief get a path from source to target
 *
 * @param root: start node
 * @param id_target: id of the target
 * @param id_src: id of the source
 * @return std::vector<std::pair<NodeEK*, EdgeEK*>>: path backtracking from
 * target to source
 *
 */
std::vector<std::pair<NodeEK*, EdgeEK*>> BFS_get_path(NodeEK* root,
                                                      int id_target,
                                                      int id_src);
/**
 * @brief breadth first search for a image graph
 *
 * @param root start node of a image graph
 * @param row image rows
 * @param col inage cols
 */
void BFS(Node* root, int rows, int cols);

/**
 * @brief deepth first search for the template class graph
 *
 * @tparam TypeNode : type of the node
 * @param root : start node of the graph
 * @param visited : true if the node has been visited
 */
template <typename TypeNode>
void DFS(TypeNode* root, std::unordered_set<TypeNode*>& visited);

/*--------------------------------------------------------
#####################implementation #####################
---------------------------------------------------------*/
template <typename TypeNode>
void DFS(TypeNode* root, std::unordered_set<TypeNode*>& visited) {
    if (!root) return;

    std::cout << "Node id: " << root->id_ << '\n';
    visited.insert(root);

    for (auto& elem : root->neighbours_) {
        if (visited.find(elem.first) == visited.end()) {
            DFS(elem.first, visited);
        }
    }
}

template <typename TypeNode>
void BFS(TypeNode* root) {
    std::unordered_set<TypeNode*> visited;

    std::queue<TypeNode*> Q;

    visited.insert(root);
    Q.push(root);

    while (!Q.empty()) {
        TypeNode* curr = Q.front();
        Q.pop();
        // std::cout << "Node id: " << curr->id_ << '\n';

        for (auto& elem : curr->neighbours_) {
            if (visited.find(elem.first) == visited.end()) {
                visited.insert(elem.first);
                Q.push(elem.first);
            }
        }
    }
}

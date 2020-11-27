#pragma once
#include "ek.h"
#include "graph.h"
#include "image_graph.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <queue>
#include <unordered_set>

void DFS(NodeEK* root, std::vector<bool>& visited);
void BFS(NodeEK* root);
void BFS(Node* root, int row, int col);
int BFS(NodeEK* root, int id_target);
std::vector<std::pair<NodeEK*, EdgeEK*>> BFS_get_path(NodeEK* root,
                                                      int id_target,
                                                      int id_src);
template <typename TypeNode>
void DFS(TypeNode* root, std::unordered_set<TypeNode*>& visited);

template <typename TypeNode>
void DFS(TypeNode* root);

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

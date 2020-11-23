#pragma once
#include "ek.h"
#include <iostream>
#include <queue>
#include <unordered_set>

void DFS(NodeEK* root, std::vector<bool>& visited);
void BFS(NodeEK* root);
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

    std::cout << "Node id: " << root->get_id() << '\n';
    visited.insert(root);

    for (auto& elem : root->get_neighbours()) {
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
        // std::cout << "Node id: " << curr->get_id() << '\n';

        for (auto& elem : curr->get_neighbours()) {
            if (visited.find(elem.first) == visited.end()) {
                visited.insert(elem.first);
                Q.push(elem.first);
            }
        }
    }
}
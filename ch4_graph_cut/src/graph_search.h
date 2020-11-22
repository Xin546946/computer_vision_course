#pragma once
#include "ek.h"
#include <unordered_set>

void DFS(Node* root, std::vector<bool>& visited);
void BFS(Node* root);
int BFS(Node* root, int id_target);
std::vector<std::pair<Node*, Edge*>> BFS_get_path(Node* root, int id_target,
                                                  int id_src);
template <typename TypeNode>
void DFS(TypeNode* root);

template <typename TypeNode>
void DFS(TypeNode* root) {
    if (!root) return;

    std::unordered_set<TypeNode*> visited;
    visited.insert(root);

    for (auto& elem : root->get_neighbours()) {
        if (visited.find(elem.first) == visited.end()) {
            DFS(elem.first, visited);
        }
    }
}
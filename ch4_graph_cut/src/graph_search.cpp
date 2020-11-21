#include "graph_search.h"
#include <iostream>

void DFS(Node* root, std::vector<bool>& visited) {
    visited[root->id_] = true;

    if (!root) return;

    for (auto& elem : root->children_) {  // (*root).children_
        if (!elem.first->id_) {
            std::cout << elem.first->id_ << '\n';
            DFS(elem.first, visited);
        }
    }
}
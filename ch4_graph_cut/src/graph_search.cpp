#include "graph_search.h"
#include <iostream>

void DFS(Node* root, std::vector<bool>& visited) {
    std::cout << root->id_ << '\n';
    visited[root->id_] = true;

    if (!root) return;

    for (auto& elem : root->children_) {  // (*root).children_
        if (!visited[elem.first->id_]) {
            DFS(elem.first, visited);
        }
    }
}

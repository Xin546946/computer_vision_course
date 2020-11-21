#include "graph_search.h"
#include <iostream>
#include <queue>
#include <unordered_set>

void DFS(Node* root, std::vector<bool>& visited) {
    if (!root) return;

    std::cout << root->id_ << '\n';
    visited[root->id_] = true;

    for (auto& elem : root->children_) {  // (*root).children_
        if (!visited[elem.first->id_]) {
            DFS(elem.first, visited);
        }
    }
}

void BFS(Node* root) {
    std::unordered_set<Node*> visited;

    std::queue<Node*> Q;
    visited.insert(root);
    Q.push(root);

    while (!Q.empty()) {
        Node* curr = Q.front();
        Q.pop();
        std::cout << curr->id_ << '\n';

        for (auto& elem : curr->children_) {
            if (visited.find(elem.first) == visited.end()) {
                visited.insert(elem.first);
                Q.push(elem.first);
            }
        }
    }
}
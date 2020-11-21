#include "graph_search.h"
#include <iostream>
#include <queue>

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

void BFS(Node* root, std::vector<bool>& visited) {
    if (!root || visited[root->id_]) return;

    std::cout << root->id_ << '\n';
    visited[root->id_] = true;

    std::queue<Node*> Q;
    Q.push(root);

    while (!Q.empty()) {
        Node* curr = Q.front();
        Q.pop();

        for (auto& elem : root->children_) {
            if (!visited[elem.first->id_]) {
                Q.push(elem.first);
            }
        }
        BFS(curr, visited);
    }
}
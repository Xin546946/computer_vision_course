#include "graph_search.h"
#include <iostream>
#include <queue>
#include <unordered_set>

void DFS(NodeEK* root, std::vector<bool>& visited) {
    if (!root) return;

    std::cout << root->id_ << '\n';
    visited[root->id_] = true;

    for (auto& elem : root->children_) {  // (*root).children_
        if (!visited[elem.first->id_]) {
            DFS(elem.first, visited);
        }
    }
}

void BFS(NodeEK* root) {
    std::unordered_set<NodeEK*> visited;

    std::queue<NodeEK*> Q;
    visited.insert(root);
    Q.push(root);

    while (!Q.empty()) {
        NodeEK* curr = Q.front();
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

std::vector<std::pair<NodeEK*, EdgeEK*>> BFS_get_path(NodeEK* root,
                                                      int id_target,
                                                      int id_src) {
    std::unordered_set<NodeEK*> visited;
    std::vector<std::pair<NodeEK*, EdgeEK*>> path;

    std::queue<NodeEK*> Q;

    visited.insert(root);
    Q.push(root);
    std::cout << "--------------- one sweep--------------- " << '\n';
    while (!Q.empty()) {
        NodeEK* curr = Q.front();

        if (curr->id_ == id_target) {
            while (curr->id_ != id_src) {
                path.emplace_back(curr, curr->parent_.second);
                std::cout << "curr id :" << curr->id_
                          << "flow :" << curr->parent_.second->flow_ << '\n';
                curr = curr->parent_.first;
            }
            /*             std::cout << "curr id :" << curr->id_
                                  << "flow :" << curr->parent_.second->flow_ <<
               '\n' */
        }

        Q.pop();

        for (auto& elem : curr->children_) {
            if (visited.find(elem.first) == visited.end() &&
                !elem.second.is_full()) {
                visited.insert(elem.first);
                Q.push(elem.first);
                elem.first->parent_ =
                    std::pair<NodeEK*, EdgeEK*>(curr, &elem.second);
            }
        }
    }
    return path;
}

/**
 * @brief
 *  travese a graph and
 *  return all the flow to node with id target
 *
 * @param root
 * @param id
 */
int BFS(NodeEK* root, int id_target) {
    int result = 0;
    std::unordered_set<NodeEK*> visited;

    std::queue<NodeEK*> Q;
    visited.insert(root);
    Q.push(root);

    while (!Q.empty()) {
        NodeEK* curr = Q.front();
        Q.pop();

        for (auto& elem : curr->children_) {
            if (visited.find(elem.first) == visited.end()) {
                if (elem.first->id_ != id_target) {
                    visited.insert(elem.first);
                }
                Q.push(elem.first);
                if (elem.first->id_ == id_target) {
                    std::cout << '\n'
                              << "node id : " << curr->id_
                              << "parent id :" << curr->parent_.first->id_
                              << "flow :" << elem.second.flow_ << '\n';
                    result += elem.second.flow_;
                }
            }
        }
    }
    return result;
}

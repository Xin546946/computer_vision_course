#pragma once
#include "ek.h"

void DFS(Node* root, std::vector<bool>& visited);
void BFS(Node* root);
int BFS(Node* root, int id_target);
std::vector<std::pair<Node*, Edge*>> BFS_get_path(Node* root, int id_target,
                                                  int id_src);
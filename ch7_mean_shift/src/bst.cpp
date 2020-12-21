#include "bst.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>

KNNResult::KNNResult(int dist, BSTNode* node) : dist_(dist), node_(node) {
}

KNNResultSet::KNNResultSet(int k) : result_set_(cmp) {
    for (int i = 0; i < k; i++) {
        result_set_.push(KNNResult());
    }
}

void KNNResultSet::add_node(KNNResult result) {
    result_set_.pop();
    result_set_.push(result);
}

int KNNResultSet::worst_dist() {
    return result_set_.top().dist_;
}

std::vector<KNNResult> KNNResultSet::get_result() {
    std::vector<KNNResult> result;
    while (!result_set_.empty()) {
        result.push_back(result_set_.top());
        result_set_.pop();
    }

    return result;
}
BSTNode::BSTNode(int value) : value_(value) {
}

BST::BST(std::vector<int> datas, bool recursively) {
    if (!recursively) {
        for (auto it = datas.begin(); it != datas.end(); it++) {
            this->add_data_recursively(*it);
        }
    } else {
        for (int data : datas) {
            this->add_data_iteratively(data);
        }
    }
}

void BST::add_data_iteratively(int data) {
    BSTNode** curr = &root_;

    while (*curr) {
        curr = data < (*curr)->value_ ? &((*curr)->smaller_) : &((*curr)->larger_);
    }

    *curr = new BSTNode(data);
}

void inorder(BSTNode* curr, std::vector<int>& result) {
    if (curr->smaller_) {
        inorder(curr->smaller_, result);
    }

    result.push_back(curr->value_);

    if (curr->larger_) {
        inorder(curr->larger_, result);
    }
}

std::vector<int> BST::inorder() {
    std::vector<int> result;
    if (root_) {
        ::inorder(root_, result);
    }
    return result;
}

void add_data(BSTNode*& root, int data) {
    if (!root) {
        root = new BSTNode(data);
    } else if (data < root->value_) {
        add_data(root->smaller_, data);
    } else {
        add_data(root->larger_, data);
    }
}

void BST::add_data_recursively(int data) {
    add_data(root_, data);
}

BSTNode* search_data_recursive(BSTNode* root, int data) {
    if (root) {
        if (data < root->value_) {
            return search_data_recursive(root->smaller_, data);
        } else if (data > root->value_) {
            return search_data_recursive(root->larger_, data);
        } else {
            return root;
        }
    }
    return nullptr;
}
BSTNode* BST::search_data_recursive(int data) {
    return ::search_data_recursive(root_, data);
}

BSTNode* BST::search_data_iterative(int data) {
    BSTNode* curr = root_;
    while (curr) {
        if (curr->value_ == data) {
            return curr;
        } else if (data < curr->value_) {
            curr = curr->smaller_;
        } else {
            curr = curr->larger_;
        }
    }
    return curr;
}

void onenn_search(BSTNode* node, int data, int min_dist, BSTNode*& min_dis_node) {
    std::cout << "Min distance is" << min_dist << '\n';

    if (node) {
        if (data > node->value_) {
            if (min_dist > data - node->value_) {
                min_dist = data - node->value_;
                min_dis_node = node;
            }
            onenn_search(node->larger_, data, min_dist, min_dis_node);
        } else if (data < node->value_) {
            if (min_dist > node->value_ - data) {
                min_dist = node->value_ - data;
                min_dis_node = node;
            }
            onenn_search(node->smaller_, data, min_dist, min_dis_node);
        } else {
            min_dis_node = node;
        }
    }
}

BSTNode* BST::onenn_search(int data) {
    int min_dist = std::numeric_limits<int>::max();
    BSTNode* min_dist_node = root_;
    if (root_) {
        ::onenn_search(root_, data, min_dist, min_dist_node);
    }
    return min_dist_node;
}

void knn_search(BSTNode* node, int data, KNNResultSet& result_set) {
    if (node) {
        // todo 单独处理= ?
        if (data < node->value_) {
            knn_search(node->smaller_, data, result_set);
            if (std::abs(node->value_ - data) <= result_set.worst_dist()) {
                knn_search(node->larger_, data, result_set);
                result_set.add_node(KNNResult(std::abs(node->value_ - data), node));
            }

        } else {
            knn_search(node->larger_, data, result_set);
            if (std::abs(data - node->value_) <= result_set.worst_dist()) {
                knn_search(node->smaller_, data, result_set);
                result_set.add_node(KNNResult(std::abs(data - node->value_), node));
            }
        }
    }
}

std::vector<KNNResult> BST::knn_search(int data, int k) {
    KNNResultSet result_set(k);
    ::knn_search(root_, data, result_set);
    return result_set.get_result();
}
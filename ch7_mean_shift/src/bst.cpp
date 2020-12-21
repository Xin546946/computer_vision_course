#include "bst.h"

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

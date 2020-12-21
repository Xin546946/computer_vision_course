#include "bst.h"

BSTNode::BSTNode(int value) : value_(value) {
}

BST::BST(std::vector<int> data) {
    for (auto it = data.begin(); it != data.end(); it++) {
        this->add_data(*it);
    }
}

void BST::add_data(int data) {
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

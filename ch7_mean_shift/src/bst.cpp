#include "bst.h"

BSTNode::BSTNode(int value) : value_(value) {
}

BST::BST(std::vector<int> data) {
    for (auto it = data.begin(); it != data.end(); it++) {
        this->add_data(*it);
    }
}

void BST::add_data(int data) {
    BSTNode* curr = new BSTNode(data);
    while (!curr) {
        if (root_ == nullptr) {
            root_ = new BSTNode(data);
        } else {
            if (data <= this->root_->value_) {
                this->root_->smaller_ = new BSTNode(data);
            } else if (data > this->root_->value_) {
                this->root_->larger_ = new BSTNode(data);
            }
        }
    }
}
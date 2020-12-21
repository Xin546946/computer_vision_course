#pragma once
#include <vector>

struct BSTNode {
    BSTNode(int value);
    BSTNode* smaller_ = nullptr;
    BSTNode* larger_ = nullptr;

    int value_;
};

class BST {
   public:
    BST(std::vector<int> data);
    std::vector<int> inorder();
    void add_data(int data);

   private:
    BSTNode* root_ = nullptr;
};
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
    BST(std::vector<int> datas, bool recursively = false);
    std::vector<int> inorder();
    void add_data_iteratively(int data);
    void add_data_recursively(int data);

   private:
    BSTNode* root_ = nullptr;
};
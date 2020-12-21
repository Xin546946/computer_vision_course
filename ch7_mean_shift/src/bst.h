#pragma once
#include <limits>
#include <queue>
#include <vector>

struct BSTNode {
    BSTNode(int value);
    BSTNode* smaller_ = nullptr;
    BSTNode* larger_ = nullptr;

    int value_;
};

struct KNNResult {
    KNNResult() = default;
    KNNResult(int dist, BSTNode* node);
    int dist_ = std::numeric_limits<int>::max();
    BSTNode* node_ = nullptr;
};

auto cmp = [](KNNResult lhs, KNNResult rhs) { return lhs.dist_ < rhs.dist_; };

class KNNResultSet {
   public:
    KNNResultSet(int k);

    void add_node(KNNResult result);
    int worst_dist();
    std::priority_queue<KNNResult, std::vector<KNNResult>, decltype(cmp)> result_set_;
};

class BST {
   public:
    BST(std::vector<int> datas, bool recursively = false);
    std::vector<int> inorder();
    void add_data_iteratively(int data);
    void add_data_recursively(int data);
    BSTNode* search_data_recursive(int data);
    BSTNode* search_data_iterative(int data);
    BSTNode* onenn_search(int data);
    std::vector<KNNResult> knn_search(int data, int k);

   private:
    BSTNode* root_ = nullptr;
};
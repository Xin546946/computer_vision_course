#pragma once
#include <array>

template <typename T, int Dim>
struct Data {
    Data() = default;
    std::array<T, Dim> data_;
};

template <typename T, int Dim>
struct KDTreeNode {
    Data<T, Dim> data_;
    int axis_;

    KDTreeNode* smaller_;
    KDTreeNode* larger_;
    std::vector<KDTreeNode*> children_;
    bool is_leaf();
};

template <typename T, int Dim>
class KDTree {
   public:
    KDTree(const std::vector<Data<T, Dim>>& data, bool recursive = true);
    KDTreeNode* search_data_recursively(const Data<T, Dim>& data);
    KDTreeNode* point_index_sort(int axis, int dim);
    std::vector<KDTreeNode*> onenn_search(const Data<T, Dim>& data);
    std::vector<KDTreeNode*> knn_search(const Data<T, Dim>& data, int k);
    std::vector<KDTreeNode*> rnn_search(const Data<T, Dim>& data, double radius);

   private:
    KDTreeNode* root_ = nullptr;
};
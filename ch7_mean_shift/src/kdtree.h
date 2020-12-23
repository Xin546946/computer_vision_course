#pragma once
#include <algorithm>
#include <array>
#include <initializer_list>
#include <vector>

template <typename T, int Dim>
struct KDTreeNode {
    KDTreeNode(const std::array<T, Dim>& data, int axis);
    std::array<T, Dim> data_;
    int axis_;

    KDTreeNode<T, Dim>* smaller_ = nullptr;
    KDTreeNode<T, Dim>* larger_ = nullptr;
    std::vector<KDTreeNode*> children_;
    bool has_leaves() {
        return (!this->children_.empty());
    };
};

template <typename T, int Dim>
class KDTree {
   public:
    // typedef typename std::vector<std::array<T, Dim>>::iterator Iter;
    typedef KDTreeNode<T, Dim>* PtrNode;
    KDTree(std::vector<std::array<T, Dim>>& data, int leaf_size = 1);
    void build_kdtree(PtrNode& curr, typename std::vector<std::array<T, Dim>>::iterator begin,
                      typename std::vector<std::array<T, Dim>>::iterator end);
    PtrNode search_data_recursively(std::array<T, Dim>& data);
    PtrNode point_index_sort(int axis, int dim);
    std::vector<PtrNode> onenn_search(const std::array<T, Dim>& data);
    std::vector<PtrNode> knn_search(const std::array<T, Dim>& data, int k);
    std::vector<PtrNode> rnn_search(const std::array<T, Dim>& data, double radius);
    std::vector<std::array<T, Dim>> inorder();

   private:
    PtrNode root_ = nullptr;
    int axis_ = 0;
    int leaf_size_;
    void next_axis();
};

/*--------------------------------------------------------
#####################implementation: KDTree #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTreeNode<T, Dim>::KDTreeNode(const std::array<T, Dim>& data, int axis) : data_(data), axis_(axis) {
}

/*--------------------------------------------------------
#####################implementation: KDTree #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTree<T, Dim>::KDTree(std::vector<std::array<T, Dim>>& data, int leaf_size) : leaf_size_(leaf_size) {
    this->build_kdtree(root_, data.begin(), data.end());
}

template <typename T, int Dim>
void KDTree<T, Dim>::build_kdtree(KDTreeNode<T, Dim>*& curr, typename std::vector<std::array<T, Dim>>::iterator begin,
                                  typename std::vector<std::array<T, Dim>>::iterator end) {
    int dist = std::distance(begin, end);

    typename std::vector<std::array<T, Dim>>::iterator mid = begin + dist / 2;
    std::nth_element(begin, mid, end,
                     [=](std::array<T, Dim>& lhs, std::array<T, Dim>& rhs) { return lhs[axis_] < rhs[axis_]; });
    curr = new KDTreeNode<T, Dim>(*mid, axis_);

    if (dist <= leaf_size_) {
        curr->children_.reserve(leaf_size_);
        std::for_each(begin, end,
                      [&](std::array<T, Dim> data) { curr->children_.push_back(new KDTreeNode<T, Dim>(data, axis_)); });
    } else {
        build_kdtree(curr->smaller_, begin, mid);
        build_kdtree(curr->larger_, mid, end);
    }

    next_axis();
}

template <typename T, int Dim>
inline void KDTree<T, Dim>::next_axis() {
    axis_ = (axis_ == Dim - 1) ? 0 : axis_ + 1;
}

template <typename T, int Dim>
void inorder(KDTreeNode<T, Dim>* curr, std::vector<std::array<T, Dim>>& result) {
    if (curr) {
        inorder<T, Dim>(curr->smaller_, result);
        // result.push_back(curr->data_);
        inorder<T, Dim>(curr->larger_, result);

        std::for_each(curr->children_.begin(), curr->children_.end(),
                      [&](KDTreeNode<T, Dim>* ptr_node) { result.push_back(ptr_node->data_); });
    }
}

template <typename T, int Dim>
std::vector<std::array<T, Dim>> KDTree<T, Dim>::inorder() {
    std::vector<std::array<T, Dim>> result;
    ::inorder<T, Dim>(root_, result);
    return result;
}

template <typename T, int Dim>
KDTreeNode<T, Dim>* search_data_recursively(KDTreeNode<T, Dim>* curr, std::array<T, Dim>& data) {
    if (curr->has_leaves()) {
        for (KDTreeNode<T, Dim>* leaf : curr->children_) {
            if (leaf->data_ == data) {
                return leaf;
            }
        }
    } else {
        int axis = curr->axis_;
        if (data[axis] < curr->data_[axis]) {
            return search_data_recursively<T, Dim>(curr->smaller_, data);
        } else if (data[axis] > curr->data_[axis]) {
            return search_data_recursively<T, Dim>(curr->larger_, data);
        } else {
            auto temp = search_data_recursively<T, Dim>(curr->smaller_, data);
            if (temp) return temp;
            temp = search_data_recursively<T, Dim>(curr->larger_, data);
            return temp;
        }
    }
    return nullptr;
}

template <typename T, int Dim>
KDTreeNode<T, Dim>* KDTree<T, Dim>::search_data_recursively(std::array<T, Dim>& data) {
    return ::search_data_recursively<T, Dim>(root_, data);
}
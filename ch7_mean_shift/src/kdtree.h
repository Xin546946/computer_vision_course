#pragma once
#include <algorithm>
#include <array>
#include <initializer_list>
#include <vector>

template <typename T, int Dim>
struct Data {
    Data(const std::initializer_list<T>& l) : data_(l);
    std::array<T, Dim> data_;
    T& operator[](int i) {
        return data[i];
    }
};

template <typename T, int Dim>
struct KDTreeNode {
    KDTreeNode(const Data<T, Dim>& data, int axis);
    Data<T, Dim> data_;
    int axis_;

    PtrNode smaller_ = nullptr;
    PtrNode larger_ = nullptr;
    std::vector<KDTreeNode*> children_;
    bool is_leaf() {
        return (this->smaller_ == nullptr && this->larger_ == nullptr);
    };
};

template <typename T, int Dim>
class KDTree {
   public:
    typedef typename std::vector<Data<T, Dim>>::iterator Iter;
    typedef KDTreeNode<T, Dim>* PtrNode;
    KDTree(const std::vector<Data<T, Dim>>& data, int leaf_size = 1);
    void build_kdtree(PtrNode& curr, Iter begin, Iter end);
    PtrNode search_data_recursively(const Data<T, Dim>& data);
    PtrNode point_index_sort(int axis, int dim);
    std::vector<PtrNode> onenn_search(const Data<T, Dim>& data);
    std::vector<PtrNode> knn_search(const Data<T, Dim>& data, int k);
    std::vector<PtrNode> rnn_search(const Data<T, Dim>& data, double radius);

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
KDTreeNode<T, Dim>::KDTreeNode(const Data<T, Dim>& data, int axis) : data_(data), axis_(axis) {
}

/*--------------------------------------------------------
#####################implementation: KDTree #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTree<T, Dim>::KDTree(const std::vector<Data<T, Dim>>& data, int leaf_size) : leaf_size_(leaf_size) {
    build_kdtree(root_, data.begin(), data.end());
}

template <typename T, int Dim>
void KDTree<T, Dim>::build_kdtree(KDTreeNode<T, Dim>*& curr, KDTree<T, Dim>::Iter begin, KDTree<T, Dim>::Iter end) {
    // choose one axis, compute the median in this axis, split the points at this axis into two pieces
    Iter mid = begin + std::distance(begin, end) / 2;
    std::nth_element(begin, mid, end, [=](Data<T, Dim>& lhs, Data<T, Dim>& rhs) { return lhs[axis_] < rhs[axis_]; });

    curr = new KDTreeNode(*mid, axis_);
    next_axis();
    if (std::distance(begin, mid) >= leaf_size) {
        build(curr->smaller_, begin, mid);
    } else {
        curr->children_.reserve(leaf_size);
        std::copy(begin, mid, std::back_insert(curr->children_));
    }
    if (std::distance(mid, end) >= leaf_size) {
        build(curr->larger_, mid, end);
    } else {
        curr->children_.reserve(leaf_size);
        std::copy(mid, end, std::back_insert(curr->children_));
    }

    // change an axis, splits the points in each side
}
template <typename T, int Dim>
inline void KDTree<T, Dim>::next_axis() {
    if (axis_ == Dim - 1) {
        axis_ = 0;
    } else {
        axis_++;
    }
}
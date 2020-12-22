#pragma once
#include <algorithm>
#include <array>
#include <initializer_list>
#include <vector>

template <typename T, int Dim>
struct Data {
    // Data(const std::initializer_list<T>& l) : data_(l) {
    // }
    std::array<T, Dim> data_;
    T& operator[](int i) {
        return data_[i];
    }
};

template <typename T, int Dim>
struct KDTreeNode {
    KDTreeNode(const Data<T, Dim>& data, int axis);
    Data<T, Dim> data_;
    int axis_;

    KDTreeNode<T, Dim>* smaller_ = nullptr;
    KDTreeNode<T, Dim>* larger_ = nullptr;
    std::vector<KDTreeNode*> children_;
    bool is_leaf() {
        return (this->smaller_ == nullptr && this->larger_ == nullptr && this->children_.empty());
    };
};

template <typename T, int Dim>
class KDTree {
   public:
    // typedef typename std::vector<Data<T, Dim>>::iterator Iter;
    typedef KDTreeNode<T, Dim>* PtrNode;
    KDTree(std::vector<Data<T, Dim>>& data, int leaf_size = 1);
    void build_kdtree(PtrNode& curr, typename std::vector<Data<T, Dim>>::iterator begin,
                      typename std::vector<Data<T, Dim>>::iterator end);
    PtrNode search_data_recursively(const Data<T, Dim>& data);
    PtrNode point_index_sort(int axis, int dim);
    std::vector<PtrNode> onenn_search(const Data<T, Dim>& data);
    std::vector<PtrNode> knn_search(const Data<T, Dim>& data, int k);
    std::vector<PtrNode> rnn_search(const Data<T, Dim>& data, double radius);
    std::vector<Data<T, Dim>> inorder();

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
KDTree<T, Dim>::KDTree(std::vector<Data<T, Dim>>& data, int leaf_size) : leaf_size_(leaf_size) {
    this->build_kdtree(root_, data.begin(), data.end());
}

template <typename T, int Dim>
void KDTree<T, Dim>::build_kdtree(KDTreeNode<T, Dim>*& curr, typename std::vector<Data<T, Dim>>::iterator begin,
                                  typename std::vector<Data<T, Dim>>::iterator end) {
    // choose one axis, compute the median in this axis, split the points at this axis into two pieces
    typename std::vector<Data<T, Dim>>::iterator mid = begin + std::distance(begin, end) / 2;
    std::nth_element(begin, mid, end, [=](Data<T, Dim>& lhs, Data<T, Dim>& rhs) { return lhs[axis_] < rhs[axis_]; });

    curr = new KDTreeNode<T, Dim>(*mid, axis_);
    next_axis();

    if (std::distance(begin, mid) >= leaf_size_) {
        build_kdtree(curr->smaller_, begin, mid);
    } else {
        curr->children_.reserve(leaf_size_);
        std::for_each(begin, mid,
                      [&](Data<T, Dim> data) { curr->children_.push_back(new KDTreeNode<T, Dim>(data, axis_)); });
    }
    if (std::distance(mid, end) >= leaf_size_) {
        build_kdtree(curr->larger_, mid, end);
    } else {
        curr->children_.reserve(leaf_size_);
        std::for_each(mid, end,
                      [&](Data<T, Dim> data) { curr->children_.push_back(new KDTreeNode<T, Dim>(data, axis_)); });
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

template <typename T, int Dim>
void inorder(KDTreeNode<T, Dim>* curr, std::vector<Data<T, Dim>>& result) {
    if (curr) {
        if (curr->children_.empty()) {
            inorder(curr->smaller_, result);
            result.push_back(curr->data_);
            inorder(curr->larger_, result);
        } else {
            std::for_each(curr->children_.begin(), curr->children_.end(),
                          [&](KDTreeNode<T, Dim>* ptr_node) { result.push_back(ptr_node->data_); });
        }
    }
}

template <typename T, int Dim>
std::vector<Data<T, Dim>> KDTree<T, Dim>::inorder() {
    std::vector<Data<T, Dim>> result;
    ::inorder(root_, result);
    return result;
}
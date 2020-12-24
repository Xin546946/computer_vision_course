#pragma once
#include <algorithm>
#include <vector>

template <typename T>
struct GKDTreeNode {
    GKDTreeNode(T* data, int axis) : data_(data), axis_(axis) {
    }
    T* data_;
    int axis_;

    GKDTreeNode<T>* smaller_ = nullptr;
    GKDTreeNode<T>* larger_ = nullptr;
    std::vector<T*> leaves_;
    bool has_leaves() {
        return (!this->leaves_.empty());
    }
};

template <typename T>
class GKdTree {
   public:
    GKdTree(T* head, int size, int leaf_size = 1);
    GKdTree() = default;
    ~GKdTree();

    std::vector<T*> rnn_search(T* data, typename T::DistType radius) const;

   private:
    void build_tree(GKDTreeNode<T>*& curr, T* begin, T* end);
    void next_axis();

    GKDTreeNode<T>* root_ = nullptr;
    int axis_ = 0;
    int leaf_size_;
    int size_;
};

/*--------------------------------------------------------
#####################implementation: GKdTree #####################
---------------------------------------------------------*/
template <typename T>
GKdTree<T>::GKdTree(T* head, int size, int leaf_size) : leaf_size_(leaf_size), size_(size) {
    this->build_tree(root_, head, head + size_);
}

template <typename T>
inline void GKdTree<T>::next_axis() {
    axis_ = (axis_ == T::dim_ - 1) ? 0 : axis_ + 1;
}

template <typename T>
void GKdTree<T>::build_tree(GKDTreeNode<T>*& curr, T* begin, T* end) {
    int dist = std::distance(begin, end);
    T* mid = begin + dist / 2;

    std::nth_element(begin, mid, end, [=](const T& lhs, const T& rhs) { return lhs[axis_] < rhs[axis_]; });
    curr = new GKDTreeNode<T>(mid, axis_);

    if (dist <= leaf_size_) {
        curr->leaves_.reserve(leaf_size_);
        std::for_each(begin, end, [&](T& data) { curr->leaves_.push_back(&data); });
    } else {
        build_tree(curr->smaller_, begin, end);
        build_tree(curr->larger_, mid, end);
    }
    next_axis();
}

template <typename T>
void inorder(GKDTreeNode<T>* curr, std::vector<GKDTreeNode<T>*>& result) {
    if (curr) {
        inorder<T>(curr->smaller_, result);

        result.push_back(curr);

        inorder<T>(curr->larger_, result);
    }
}

template <typename T>
void rnn_search(GKDTreeNode<T>* curr, T* data, std::vector<T*>& result_set, typename T::DistType radius) {
    if (curr->has_leaves()) {
        for (T* child : curr->leaves_) {
            if (T::is_in_radius(data, child, radius * radius)) {
                result_set.push_back(child);
            }
        }
    } else {
        if ((*data)[curr->axis_] < (*curr->data_)[curr->axis_]) {
            rnn_search<T>(curr->smaller_, data, result_set, radius);

            if (T::is_in_radius(data, curr->data_, curr->axis_, radius)) {
                rnn_search<T>(curr->larger_, data, result_set, radius);
            }
        } else if ((*data)[curr->axis_] > (*curr->data_)[curr->axis_]) {
            rnn_search<T>(curr->larger_, data, result_set, radius);
            if (T::is_in_radius(data, curr->data_, curr->axis_, radius)) {
                rnn_search<T>(curr->smaller_, data, result_set, radius);
            }
        } else {
            rnn_search<T>(curr->smaller_, data, result_set, radius);
            rnn_search<T>(curr->larger_, data, result_set, radius);
        }
    }
}

template <typename T>
std::vector<T*> GKdTree<T>::rnn_search(T* data, typename T::DistType radius) const {
    std::vector<T*> result_set;
    ::rnn_search<T>(root_, data, result_set, radius);
    return result_set;
}
template <typename T>
GKdTree<T>::~GKdTree() {
    std::vector<GKDTreeNode<T>*> ptr_nodes;
    ptr_nodes.reserve(size_);
    ::inorder(root_, ptr_nodes);

    for (auto ptr_node : ptr_nodes) {
        delete ptr_node;
        ptr_node = nullptr;
    }
}
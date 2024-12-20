#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>
template <typename T, int Dim>
struct KDTreeNode {
    typedef std::array<T, Dim> KDData;
    typedef KDTreeNode<T, Dim>* PtrNode;

    KDTreeNode(const KDData& data, int axis);
    KDData data_;
    int axis_;

    PtrNode smaller_ = nullptr;
    PtrNode larger_ = nullptr;
    std::vector<PtrNode> children_;
    bool has_leaves() {
        return (!this->children_.empty());
    };
};

template <typename T, int Dim>
struct KNNResult {
    KNNResult() = default;
    KNNResult(int dist, KDTreeNode<T, Dim>* node) : dist_(dist), node_(node) {
    }
    T dist_ = std::numeric_limits<T>::max();
    KDTreeNode<T, Dim>* node_ = nullptr;
};

template <typename T, int Dim>
inline auto cmp = [](const KNNResult<T, Dim>& lhs, const KNNResult<T, Dim>& rhs) { return lhs.dist_ < rhs.dist_; };

template <typename T, int Dim>
class KNNResultSet {
   public:
    typedef typename KDTreeNode<T, Dim>::PtrNode PtrNode;
    KNNResultSet(int k);

    void add_node(T dist, PtrNode node);
    T worst_dist();
    std::priority_queue<KNNResult<T, Dim>, std::vector<KNNResult<T, Dim>>, decltype(cmp<T, Dim>)> result_set_;
    std::vector<KNNResult<T, Dim>> get_result();
};

template <typename T, int Dim>
class RNNResultSet {
   public:
    typedef typename KDTreeNode<T, Dim>::PtrNode PtrNode;
    RNNResultSet(T radius) : radius_(radius), radius_square_(radius * radius) {
    }
    void add_node(PtrNode node);
    std::vector<std::array<T, Dim>> get_result() const;
    std::vector<PtrNode> result_set_;
    T radius_;
    T radius_square_;
};

template <typename T, int Dim>
class KDTree {
   public:
    typedef typename KDTreeNode<T, Dim>::KDData KDData;
    typedef typename KDTreeNode<T, Dim>::PtrNode PtrNode;
    typedef typename std::vector<std::array<T, Dim>>::iterator IterData;

    KDTree(std::vector<KDData>& data, int leaf_size = 1);
    KDTree() = default;
    ~KDTree();
    PtrNode search_data_recursively(const KDData& data) const;
    // PtrNode point_index_sort(int axis, int dim);
    // std::vector<PtrNode> onenn_search(const std::array<T, Dim>& data);
    KNNResultSet<T, Dim> knn_search(const KDData& data, int k) const;
    RNNResultSet<T, Dim> rnn_search(const KDData& data, T radius) const;
    std::vector<KDData> inorder(bool only_leaf = false) const;

   private:
    void build_kdtree(PtrNode& curr, IterData begin, IterData end);
    PtrNode root_ = nullptr;
    int axis_ = 0;
    int leaf_size_;
    int size_data_;
    void next_axis();
};
template <typename T, int Dim>
void inorder(KDTreeNode<T, Dim>* curr, std::vector<KDTreeNode<T, Dim>*>& result, bool only_leaf);

/*--------------------------------------------------------
#####################implementation: KDTreeNode #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTreeNode<T, Dim>::KDTreeNode(const KDData& data, int axis) : data_(data), axis_(axis) {
}

/*--------------------------------------------------------
#####################implementation: KNNResultSet #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KNNResultSet<T, Dim>::KNNResultSet(int k) : result_set_(cmp<T, Dim>) {
    for (int i = 0; i < k; i++) {
        result_set_.push(KNNResult<T, Dim>());
    }
}

template <typename T, int Dim>
inline void KNNResultSet<T, Dim>::add_node(T dist, PtrNode node) {
    assert(std::abs(dist) >= -static_cast<T>(1e-6));

    result_set_.pop();
    result_set_.emplace(dist, node);
}

template <typename T, int Dim>
inline T KNNResultSet<T, Dim>::worst_dist() {
    return result_set_.top().dist_;
}

template <typename T, int Dim>
std::vector<KNNResult<T, Dim>> KNNResultSet<T, Dim>::get_result() {
    std::vector<KNNResult<T, Dim>> result;
    while (!result_set_.empty()) {
        result.push_back(result_set_.top());
        result_set_.pop();
    }
    return result;
}

/*--------------------------------------------------------
#####################implementation: RNNResultSet #####################
---------------------------------------------------------*/
template <typename T, int Dim>
inline void RNNResultSet<T, Dim>::add_node(PtrNode node) {
    result_set_.push_back(node);
}
template <typename T, int Dim>
std::vector<std::array<T, Dim>> RNNResultSet<T, Dim>::get_result() const {
    std::vector<std::array<T, Dim>> result;
    for (PtrNode ptr_node : result_set_) {
        result.push_back(ptr_node->data_);
    }
    return result;
}
/*--------------------------------------------------------
#####################implementation: KDTree #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTree<T, Dim>::KDTree(std::vector<KDData>& data, int leaf_size) : leaf_size_(leaf_size), size_data_(data.size()) {
    this->build_kdtree(root_, data.begin(), data.end());
}

template <typename T, int Dim>
KDTree<T, Dim>::~KDTree() {
    std::vector<PtrNode> ptr_nodes;
    ptr_nodes.reserve(size_data_);
    ::inorder(root_, ptr_nodes, false);

    for (auto ptr_node : ptr_nodes) {
        delete ptr_node;
        ptr_node = nullptr;
    }
}

template <typename T, int Dim>
void KDTree<T, Dim>::build_kdtree(PtrNode& curr, IterData begin, IterData end) {
    int dist = std::distance(begin, end);

    IterData mid = begin + dist / 2;
    std::nth_element(begin, mid, end, [=](const KDData& lhs, const KDData& rhs) { return lhs[axis_] < rhs[axis_]; });
    curr = new KDTreeNode<T, Dim>(*mid, axis_);

    next_axis();

    if (dist <= leaf_size_) {
        curr->children_.reserve(leaf_size_);
        std::for_each(begin, end,
                      [&](const KDData& data) { curr->children_.push_back(new KDTreeNode<T, Dim>(data, axis_)); });
    } else {
        build_kdtree(curr->smaller_, begin, mid);
        build_kdtree(curr->larger_, mid, end);
    }
}

template <typename T, int Dim>
inline void KDTree<T, Dim>::next_axis() {
    axis_ = (axis_ == Dim - 1) ? 0 : axis_ + 1;
}

template <typename T, int Dim>
void inorder(KDTreeNode<T, Dim>* curr, std::vector<KDTreeNode<T, Dim>*>& result, bool only_leaf) {
    if (curr) {
        inorder<T, Dim>(curr->smaller_, result, only_leaf);

        if (!only_leaf) {
            result.push_back(curr);
        }
        inorder<T, Dim>(curr->larger_, result, only_leaf);

        std::for_each(curr->children_.begin(), curr->children_.end(),
                      [&](KDTreeNode<T, Dim>* ptr_node) { result.push_back(ptr_node); });
    }
}

template <typename T, int Dim>
std::vector<typename KDTree<T, Dim>::KDData> KDTree<T, Dim>::inorder(bool only_leaf) const {
    std::vector<PtrNode> ptr_nodes;
    ptr_nodes.reserve(size_data_);
    ::inorder<T, Dim>(root_, ptr_nodes, only_leaf);
    std::vector<KDData> result;
    result.reserve(size_data_);

    for (auto ptr_node : ptr_nodes) {
        result.push_back(ptr_node->data_);
    }

    return result;
}

template <typename T, int Dim>
KDTreeNode<T, Dim>* search_data_recursively(KDTreeNode<T, Dim>* curr, const std::array<T, Dim>& data) {
    if (curr->has_leaves()) {
        for (KDTreeNode<T, Dim>* leaf : curr->children_) {
            if (leaf->data_ == data) return leaf;
        }
    } else {
        int axis = curr->axis_;
        if (data[axis] < curr->data_[axis]) {
            return search_data_recursively<T, Dim>(curr->smaller_, data);
        } else if (data[axis] > curr->data_[axis]) {
            return search_data_recursively<T, Dim>(curr->larger_, data);
        } else {
            auto ptr1 = search_data_recursively<T, Dim>(curr->smaller_, data);
            if (ptr1) {
                return ptr1;
            }

            auto ptr2 = search_data_recursively<T, Dim>(curr->larger_, data);

            if (ptr2) {
                return ptr2;
            }
        }
    }
    return nullptr;
}

template <typename T, int Dim>
typename KDTree<T, Dim>::PtrNode KDTree<T, Dim>::search_data_recursively(const KDData& data) const {
    return ::search_data_recursively<T, Dim>(root_, data);
}

template <typename T, int Dim>
inline T compute_square_distance(const std::array<T, Dim>& lhs, const std::array<T, Dim>& rhs) {
    T dist = static_cast<T>(0.0);
    for (int i = 0; i < Dim; i++) {
        dist += std::pow(lhs[i] - rhs[i], static_cast<T>(2.0));
    }
    return dist;
}

template <typename T, int Dim>
void knn_search(KDTreeNode<T, Dim>* curr, const std::array<T, Dim>& data, KNNResultSet<T, Dim>& result_set) {
    if (curr->has_leaves()) {
        for (KDTreeNode<T, Dim>* child : curr->children_) {
            T dist = compute_square_distance<T, Dim>(child->data_, data);
            if (dist <= result_set.worst_dist()) {
                result_set.add_node(dist, child);
            }
        }
    } else {
        if (data[curr->axis_] < curr->data_[curr->axis_]) {
            knn_search<T, Dim>(curr->smaller_, data, result_set);

            if (std::abs(data[curr->axis_] - curr->data_[curr->axis_]) <= result_set.worst_dist()) {
                knn_search<T, Dim>(curr->larger_, data, result_set);
            }

        } else if (data[curr->axis_] > curr->data_[curr->axis_]) {
            knn_search<T, Dim>(curr->larger_, data, result_set);

            if (std::abs(data[curr->axis_] - curr->data_[curr->axis_]) <= result_set.worst_dist()) {
                knn_search<T, Dim>(curr->smaller_, data, result_set);
            }
        } else {
            knn_search<T, Dim>(curr->smaller_, data, result_set);
            knn_search<T, Dim>(curr->larger_, data, result_set);
        }
    }
}

template <typename T, int Dim>
KNNResultSet<T, Dim> KDTree<T, Dim>::knn_search(const KDData& data, int k) const {
    KNNResultSet<T, Dim> result_set(k);
    ::knn_search<T, Dim>(root_, data, result_set);

    return result_set;
}

template <typename T, int Dim>
void rnn_search(KDTreeNode<T, Dim>* curr, const std::array<T, Dim>& data, RNNResultSet<T, Dim>& result_set) {
    if (curr->has_leaves()) {
        for (KDTreeNode<T, Dim>* child : curr->children_) {
            // std::cout << child->data_[0] << " ";
            T dist = compute_square_distance<T, Dim>(child->data_, data);
            if (dist <= result_set.radius_square_) {
                result_set.add_node(child);
            }
        }
    } else {
        if (data[curr->axis_] < curr->data_[curr->axis_]) {
            rnn_search<T, Dim>(curr->smaller_, data, result_set);

            if (std::abs(data[curr->axis_] - curr->data_[curr->axis_]) <= result_set.radius_) {
                rnn_search<T, Dim>(curr->larger_, data, result_set);
            }

        } else if (data[curr->axis_] > curr->data_[curr->axis_]) {
            rnn_search<T, Dim>(curr->larger_, data, result_set);

            if (std::abs(data[curr->axis_] - curr->data_[curr->axis_]) <= result_set.radius_) {
                rnn_search<T, Dim>(curr->smaller_, data, result_set);
            }
        } else {
            rnn_search<T, Dim>(curr->smaller_, data, result_set);
            rnn_search<T, Dim>(curr->larger_, data, result_set);
        }
    }
}

template <typename T, int Dim>
RNNResultSet<T, Dim> KDTree<T, Dim>::rnn_search(const KDData& data, T radius) const {
    RNNResultSet<T, Dim> result_set(radius);
    ::rnn_search<T, Dim>(root_, data, result_set);

    return result_set;
}

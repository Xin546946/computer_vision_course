#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <queue>
#include <vector>
template <typename T, int Dim>
struct KDTreeNode {
    typedef std::array<T, Dim> KdData;
    typedef KDTreeNode<T, Dim>* PtrNode;

    KDTreeNode(const KdData& data, int axis);
    KdData data_;
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
    int dist_ = std::numeric_limits<int>::max();
    KDTreeNode<T, Dim>* node_ = nullptr;
};

template <typename T, int Dim>
auto cmp = [](KNNResult<T, Dim> lhs, KNNResult<T, Dim> rhs) { return lhs.dist_ < rhs.dist_; };

template <typename T, int Dim>
class KNNResultSet {
   public:
    KNNResultSet(int k);

    void add_node(int dist, KDTreeNode<T, Dim>* node);
    int worst_dist();
    std::priority_queue<KNNResult<T, Dim>, std::vector<KNNResult<T, Dim>>, decltype(cmp<T, Dim>)> result_set_;
    std::vector<KNNResult<T, Dim>> get_result();
};

template <typename T, int Dim>
class KDTree {
   public:
    typedef typename KDTreeNode<T, Dim>::KdData KdData;
    typedef typename KDTreeNode<T, Dim>::PtrNode PtrNode;
    typedef typename std::vector<std::array<T, Dim>>::iterator IterNode;

    KDTree(std::vector<KdData>& data, int leaf_size = 1);

    PtrNode search_data_recursively(const KdData& data) const;
    PtrNode point_index_sort(int axis, int dim);
    // std::vector<PtrNode> onenn_search(const std::array<T, Dim>& data);
    KNNResultSet<T, Dim> knn_search(std::array<T, Dim>& data, int k);
    KNNResultSet<T, Dim> rnn_search(std::array<T, Dim>& data, double radius);
    std::vector<KdData> inorder() const;

   private:
    void build_kdtree(PtrNode& curr, IterNode begin, IterNode end);
    PtrNode root_ = nullptr;
    int axis_ = 0;
    int leaf_size_;
    int size_data_;
    void next_axis();
};

/*--------------------------------------------------------
#####################implementation: KDTreeNode #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTreeNode<T, Dim>::KDTreeNode(const KdData& data, int axis) : data_(data), axis_(axis) {
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
void KNNResultSet<T, Dim>::add_node(int dist, KDTreeNode<T, Dim>* node) {
    result_set_.pop();
    result_set_.emplace(dist, node);
}

template <typename T, int Dim>
int KNNResultSet<T, Dim>::worst_dist() {
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
#####################implementation: KDTree #####################
---------------------------------------------------------*/
template <typename T, int Dim>
KDTree<T, Dim>::KDTree(std::vector<KdData>& data, int leaf_size) : leaf_size_(leaf_size), size_data_(data.size()) {
    this->build_kdtree(root_, data.begin(), data.end());
}

template <typename T, int Dim>
void KDTree<T, Dim>::build_kdtree(PtrNode& curr, IterNode begin, IterNode end) {
    int dist = std::distance(begin, end);

    IterNode mid = begin + dist / 2;
    std::nth_element(begin, mid, end, [=](const KdData& lhs, const KdData& rhs) { return lhs[axis_] < rhs[axis_]; });
    curr = new KDTreeNode<T, Dim>(*mid, axis_);

    if (dist <= leaf_size_) {
        curr->children_.reserve(leaf_size_);
        std::for_each(begin, end,
                      [&](const KdData& data) { curr->children_.push_back(new KDTreeNode<T, Dim>(data, axis_)); });
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
std::vector<typename KDTree<T, Dim>::KdData> KDTree<T, Dim>::inorder() const {
    std::vector<KdData> result;
    result.reserve(size_data_);
    ::inorder<T, Dim>(root_, result);
    return result;
}

template <typename T, int Dim>
KDTreeNode<T, Dim>* search_data_recursively(KDTreeNode<T, Dim>* curr, const std::array<T, Dim>& data) {
    if (curr->has_leaves()) {
        for (KDTreeNode<T, Dim>* leaf : curr->children_) {
            bool equal = true;
            for (int i = 0; i < Dim; i++) {
                if (leaf->data_[i] != data[i]) {
                    equal = false;
                    break;
                }
            }
            if (equal) return leaf;
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
typename KDTree<T, Dim>::PtrNode KDTree<T, Dim>::search_data_recursively(const KdData& data) const {
    return ::search_data_recursively<T, Dim>(root_, data);
}

template <typename T, int Dim>
T compute_square_distance(const std::array<T, Dim>& lhs, const std::array<T, Dim>& rhs) {
    T dist;
    for (int i = 0; i < Dim; i++) {
        dist += std::pow(lhs[i] - rhs[i], 2);
    }
    return dist;
}

template <typename T, int Dim>
void knn_search(KDTreeNode<T, Dim>* curr, std::array<T, Dim>& data, KNNResultSet<T, Dim>& result_set) {
    if (curr->has_leaves()) {
        for (KDTreeNode<T, Dim>* child : curr->children_) {
            T dist = compute_square_distance<T, Dim>(child->data_, data);
            result_set.add_node(dist, child);
        }
    }
    if (data[curr->axis_] < curr->data_[curr->axis_]) {
        knn_search<T, Dim>(curr->smaller_, data, result_set);
        // todo implement worst dist
        if (std::abs(data[curr->axis_] - curr->data_[curr->axis_]) < result_set.worst_dist()) {
            knn_search<T, Dim>(curr->larger_, data, result_set);
        }
    } else if (data[curr->axis_] > curr->data_[curr->axis_]) {
        knn_search<T, Dim>(curr->larger_, data, result_set);
        if (std::abs(data[curr->axis_] - curr->data_[curr->axis_] < result_set.worst_dist())) {
            knn_search<T, Dim>(curr->smaller_, data, result_set);
        }
    } else {
        result_set.add_node(0, curr);
        knn_search<T, Dim>(curr->smaller_, data, result_set);
        knn_search<T, Dim>(curr->larger_, data, result_set);
    }
}

template <typename T, int Dim>
KNNResultSet<T, Dim> KDTree<T, Dim>::knn_search(std::array<T, Dim>& data, int k) {
    KNNResultSet<T, Dim> result_set(k);
    ::knn_search<T, Dim>(root_, data, result_set);

    return result_set;
}
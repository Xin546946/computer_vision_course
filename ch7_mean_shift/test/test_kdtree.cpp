#include "kdtree.h"
#include "tictoc.h"
#include <array>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <vector>

std::vector<int> generate_random_data(int num, int min, int max) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_int_distribution<int> uniform_dist(min, max);
    std::vector<int> data(num);
    for (int i = 0; i < num; i++) {
        data[i] = uniform_dist(gen);
    }
    return data;
}

std::vector<float> generate_random_data(int num, float min, float max) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<float> uniform_dist(min, max);
    std::vector<float> data(num);
    for (int i = 0; i < num; i++) {
        data[i] = uniform_dist(gen);
    }
    return data;
}

float generate_random_data(float min, float max) {
    return generate_random_data(1, min, max)[0];
}

int generate_random_data(int min, int max) {
    return generate_random_data(1, min, max)[0];
}

template <int Dim>
std::vector<std::array<int, Dim>> generate_nd_data(int num, int min, int max) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::array<std::uniform_int_distribution<int>, Dim> uniform_dists;
    for (int i = 0; i < Dim; i++) {
        uniform_dists[i] = std::uniform_int_distribution<int>(min, max);
    }

    std::vector<std::array<int, Dim>> result(num);
    for (int n = 0; n < num; n++) {
        std::array<int, Dim> data;
        for (int i = 0; i < Dim; i++) {
            data[i] = uniform_dists[i](gen);
        }
        result[n] = (data);
    }

    return result;
}

template <int Dim>
std::vector<std::array<int, Dim>> generate_3d_data() {
    std::vector<std::array<int, 3>> data = generate_nd_data<3>(1e6, 0, 1e9);
    // for (auto d : data) {
    //     std::cout << d[0] << " " << d[1] << " " << d[2] << '\n';
    // }
    std::vector<std::array<int, 3>> data_test;
    for (auto d : data) {
        std::array<int, 3> d2;
        d2[0] = d[0];
        d2[1] = d[1];
        d2[2] = d[2];
        data_test.push_back(d2);
    }
    return data_test;
}

//! test for generate n dim random data
void test_3dtree_with_diff_leaf_size() {
    for (int leaf_size = 1; leaf_size < 1e8; leaf_size *= 10) {
        std::vector<std::array<int, 3>> data_test = generate_3d_data<3>();
        std::cout << "@@@@@@ Search for leaf size " << leaf_size << '\n';
        KDTree<int, 3> kdtree(data_test, leaf_size);
        tictoc::tic();
        KDTreeNode<int, 3>* node = kdtree.search_data_recursively(data_test[1000]);
        std::cout << "seatch data costs " << tictoc::toc() / 1e3 << "ms\n";
        if (node) {
            std::cout << node->data_[0] << " " << node->data_[1] << " " << node->data_[2] << '\n';
            std::cout << data_test[1000][0] << " " << data_test[1000][1] << " " << data_test[1000][2] << '\n';
            assert(node->data_ == data_test[1000]);
        } else {
            std::cout << "there is no such a data in kd tree." << '\n';
        }
    }
}

int main(int argc, char** argv) {
    //! test for add and traverse data

    // std::array<int, 1> d_search;
    // d_search[0] = -1;
    // for (int leaf_size = 1; leaf_size < 1e4; leaf_size *= 10) {
    //     std::vector<int> data = generate_random_data(1e6, 0, 1e9);
    //     std::vector<std::array<int, 1>> data_test;

    //     for (auto d : data) {
    //         std::array<int, 1> d2;
    //         d2[0] = d;
    //         data_test.push_back(d2);
    //     }

    //     tictoc::tic();
    //     KDTree<int, 1> kdtree(data_test, leaf_size);

    //     std::cout << "build kdtree with leaf size :" << leaf_size << " cost time :" << tictoc::toc() / 1e3 << "
    //     ms\n";

    //     std::vector<std::array<int, 1>> result = kdtree.inorder();

    //     tictoc::tic();
    //     KDTreeNode<int, 1>* node = kdtree.search_data_recursively(d_search);
    //     if (node) {
    //         assert(node->data_[0] == d_search[0]);
    //     } else {
    //         std::cout << "there is no such a data in kd tree." << '\n';
    //     }
    //     std::cout << "seatch an unexisting data costs " << tictoc::toc() / 1e3 << "ms\n";
    // }

    std::array<int, 1> search_data{10};
    // brute force search

    std::vector<std::array<int, 1>> data_test{{5}, {1}, {2}, {3}, {4}, {100}};
    KDTree<int, 1> kdtree(data_test, 1);
    std::vector<std::array<int, 1>> inorder_data = kdtree.inorder();
    for (auto i : inorder_data) {
        std::cout << i[0] << '\n';
    }
    KDTreeNode<int, 1>* d = kdtree.search_data_recursively(std::array<int, 1>{1});
    std::cout << d->data_[0] << '\n';
    std::cout << "Success to build a tree" << '\n';

    KNNResultSet<int, 1> knn_result = kdtree.knn_search(search_data, 2);
    for (auto r : knn_result.get_result()) {
        std::cout << r.node_->data_[0] << '\n';
    }
    return 0;
}

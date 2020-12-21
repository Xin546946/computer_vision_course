#include "bst.h"
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

int main(int argc, char** argv) {
    // test for add and traverse data
    // for (int n = 100; n < 1e5; n *= 10) {
    //     std::vector<int> data = generate_random_data(n, 0, 1e9);

    //     tictoc::tic();
    //     BST bst(data, false);
    //     std::cout << "iteratively add " << n << " data cost :" << tictoc::toc() / 1e6 << " seconds\n";

    //     tictoc::tic();
    //     BST bst2(data, true);
    //     std::cout << "recursively add " << n << " data cost : " << tictoc::toc() / 1e6 << " seconds\n ";

    //     tictoc::tic();
    //     std::vector<int> data_inorder = bst.inorder();
    //     std::cout << "recursively inorder traverse " << n << " data cost " << tictoc::toc() / 1e6 << " seconds\n";

    //     std::sort(data.begin(), data.end());

    //     assert(data == data_inorder);
    // }

    // test for search data
    // std::vector<int> data;
    // for (int i = 0; i < 1e6; i++) {
    //     data.push_back(i);
    // }
    // std::cout << " Generate data done! " << '\n';
    // tictoc::tic();
    // BST bst(data, true);
    // std::cout << "Build BST costs " << tictoc::toc() / 1e3 << " miliseconds\n";
    // int query_data = 1e5;
    // tictoc::tic();
    // BSTNode* node_for_search = bst.search_data_iterative(query_data);
    // std::cout << "Search the data iterative cost: " << tictoc::toc() / 1e3 << " miliseconds\n";
    // if (node_for_search) {
    //     assert(node_for_search->value_ == query_data);
    //     std::cout << "Has search " << node_for_search->value_ << " iterative successfully" << '\n';
    // } else {
    //     std::cout << "No data in the BST " << '\n';
    // }

    // tictoc::tic();
    // BSTNode* node_for_search_recur = bst.search_data_recursive(query_data);
    // std::cout << "Search the data recursive cost: " << tictoc::toc() / 1e3 << " miliseconds\n";
    // if (node_for_search_recur) {
    //     assert(node_for_search_recur->value_ == query_data);
    //     std::cout << "Has search " << node_for_search_recur->value_ << "recursive  successfully" << '\n';
    // } else {
    //     std::cout << "No data in the BST " << '\n';
    // }
    // test for 1 nn search
    // std::vector<int> data = generate_random_data(1000, 0, 10000);
    // BST bst(data);
    // BSTNode* node = bst.onenn_search(45);
    // if (node) {
    //     std::cout << node->value_ << '\n';
    // } else {
    //     std::cout << "BST is empty! " << '\n';
    // }

    // test for knn search
    std::vector<int> data{1, 2, 3, 4, 6, 7};
    BST bst(data);
    std::vector<KNNResult> result = bst.knn_search(4, 2);
    for (auto r : result) {
        std::cout << r.node_->value_ << '\n';
    }
    return 0;
}
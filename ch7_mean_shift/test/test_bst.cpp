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
    //! test for add and traverse data
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
    //! test for 1 nn search
    // std::vector<int> data = generate_random_data(1000, 0, 10000);
    // BST bst(data);
    // BSTNode* node = bst.onenn_search(45);
    // if (node) {
    //     std::cout << node->value_ << '\n';
    // } else {
    //     std::cout << "BST is empty! " << '\n';
    // }

    //! test for knn search with compare to brute force
    std::vector<int> data = generate_random_data(1000000, 0, 100000000000);
    int dist;
    std::cout << "Search 100,000,000,000 random points for comparison of brute force and RNN\n";
    std::cout << "With center 1000000 radius 50000 \n";
    std::cout << "@@@@@@@@@@@ Brute force" << '\n';
    std::vector<int> brute_force_data;
    tictoc::tic();
    for (int d : data) {
        if (std::abs(d - 100000) <= 50000) {
            brute_force_data.push_back(d);
        }
    }
    std::cout << "Brute force needs: " << tictoc::toc() / 1e3 << " miliseconds\n";
    tictoc::tic();
    BST bst(data);
    std::cout << "Build a bst needs: " << tictoc::toc() / 1e3 << " miliseconds\n";
    std::cout << "@@@@@@@@@@@ RNN Search" << '\n';
    tictoc::tic();
    std::vector<BSTNode*> result = bst.rnn_search(100000, 50000);
    std::cout << "RNN Search needs: " << tictoc::toc() / 1e3 << " miliseconds\n";
    std::cout << "RNN Search result" << '\n';
    auto cmp_result = [](BSTNode* lhs, BSTNode* rhs) { return lhs->value_ < rhs->value_; };
    std::sort(result.begin(), result.end(), cmp_result);
    std::sort(brute_force_data.begin(), brute_force_data.end());
    assert(result.size() == brute_force_data.size());
    for (int i = 0; i < brute_force_data.size(); i++) {
        assert(brute_force_data[i] == result[i]->value_);
    }

    return 0;
}

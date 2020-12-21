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
    for (int n = 100; n < 1e9; n *= 10) {
        std::vector<int> data = generate_random_data(n, 0, 1e9);

        tictoc::tic();
        BST bst(data, false);
        std::cout << "iteratively add " << n << " data cost :" << tictoc::toc() / 1e6 << " seconds\n";

        tictoc::tic();
        BST bst2(data, true);
        std::cout << "recursively add " << n << " data cost : " << tictoc::toc() / 1e6 << " seconds\n ";

        tictoc::tic();
        std::vector<int> data_inorder = bst.inorder();
        std::cout << "recursively inorder traverse " << n << " data cost " << tictoc::toc() / 1e6 << " seconds\n";

        std::sort(data.begin(), data.end());

        assert(data == data_inorder);
    }
    return 0;
}
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

int main(int argc, char** argv) {
    //! test for add and traverse data

    for (int leaf_size = 1; leaf_size < 1e4; leaf_size *= 10) {
        std::vector<int> data = generate_random_data(1e6, 0, 1e9);
        std::vector<Data<int, 1>> data_test;

        for (auto d : data) {
            Data<int, 1> d2;
            d2[0] = d;
            data_test.push_back(d2);
        }

        tictoc::tic();
        KDTree<int, 1> kdtree(data_test, leaf_size);
        std::cout << "build kdtree with leaf size :" << leaf_size << " cost time :" << tictoc::toc() / 1e3 << " ms\n";

        std::vector<Data<int, 1>> result = kdtree.inorder();
    }
    return 0;
}

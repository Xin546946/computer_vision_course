#include "bst.h"
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
    int num = 20;
    std::vector<int> data = generate_random_data(20, 0, 100);
    for (int d : data) {
        std::cout << d << '\n';
    }
    BST bst(data);
    std::vector<int> data_inorder = bst.inorder();
    std::sort(data.begin(), data.end());

    for (size_t i = 0; i < data.size(); i++) {
        std::cout << "@@@ :" << data[i] << "  #####" << data_inorder[i] << '\n';
    }

    assert(data == data_inorder);

    bst.add_data(generate_random_data(0, 10));

    return 0;
}
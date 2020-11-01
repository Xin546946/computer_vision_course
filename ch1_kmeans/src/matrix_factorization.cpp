#include <ctime>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

void non_negative_matrix_factorization(Eigen::MatrixXf V, Eigen::MatrixXf H,
                                       Eigen::MatrixXf W, int max_iteration,
                                       int r);

int main() {
    Eigen::MatrixXf V;
    Eigen::MatrixXf H;
    Eigen::MatrixXf W;

    // V = Eigen::MatrixXf(5, 8);
    // W = Eigen::MatrixXf(5, 2);
    // H = Eigen::MatrixXf(2, 8);

    // V.setRandom();
    // V = V.array().abs() * 255;
    V = ((Eigen::MatrixXf::Random(5, 8) + Eigen::MatrixXf::Ones(5, 8)) *
         122.5f);

    // Eigen::MatrixXi X = V.cast<int>();
    // std::cout << X << std::endl;
    int max_iteration = 100;
    int r = 2;
    non_negative_matrix_factorization(V, W, H, max_iteration, r);
    std::cout << H << std::endl;
    std::cout << W << std::endl;
    return 0;
}

void non_negative_matrix_factorization(Eigen::MatrixXf V, Eigen::MatrixXf W,
                                       Eigen::MatrixXf H,
                                       const int max_iteration, const int r) {
    int m = V.rows();
    int n = V.cols();

    // Create H matrix with randomized initial values
    W = Eigen::MatrixXf(m, r);
    W.setRandom();
    W.array().abs();
    W = ((Eigen::MatrixXf::Random(m, r) + Eigen::MatrixXf::Ones(m, r)) *
         122.5f);
    // Create our W with randomiyed initial values
    H = Eigen::MatrixXf(r, n);
    H.setRandom();
    H.array().abs();
    H = ((Eigen::MatrixXf::Random(r, n) + Eigen::MatrixXf::Ones(r, n)) *
         122.5f);
    std::cout << W * H << std::endl;
    // plot H and W
    // Create an identity matirx
    // Eigen::MatrixXf ones = Eigen::MatrixXf::Constant(n, m, 1.0);
    float epsilon = 0.0001f;

    // Run the multiplicate update rules for max_iteration iterations
    for (int i = 0; i < max_iteration; i++) {
        // update H
        Eigen::MatrixXf WTV = W.transpose() * V;       //  r*n
        Eigen::MatrixXf WTWH = W.transpose() * W * H;  // r*n
        Eigen::MatrixXf step_length_H = WTV.array() / (WTWH.array() + epsilon);
        H = H.array() * step_length_H.array();
        // std::cout << H << std::endl;
        // std::cout << W << std::endl;
        // update w
        Eigen::MatrixXf VHT = V * H.transpose();
        Eigen::MatrixXf WHHT = W * H * H.transpose();
        Eigen::MatrixXf step_length_W = VHT.array() / (WHHT.array() + epsilon);
        W = W.array() * step_length_W.array();
        // std::cout << W    << std::endl;
        float loss = (V - W * H).array().square().sum();
        // std::cout << "norm(V-WH) is : " << loss << '\n';
    }
    std::cout << V << std::endl;
    std::cout << "************" << std::endl;
    std::cout << W * H << std::endl;
    std::cout << "************" << std::endl;
    std::cout << V - W * H << std::endl;
}

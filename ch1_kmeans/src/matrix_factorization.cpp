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
    V = ((Eigen::MatrixXf::Random(1024, 64) + Eigen::MatrixXf::Ones(1024, 64)) *
         122.5f);
    // Eigen::MatrixXi X = V.cast<int>();
    // std::cout << X << std::endl;
    int max_iteration = 1000;
    int r = 8;
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

    // Create our W with randomiyed initial values
    H = Eigen::MatrixXf(r, n);
    H.setRandom();
    H.array().abs();

    // plot H and W
    // Create an identity matirx
    Eigen::MatrixXf ones = Eigen::MatrixXf::Constant(n, m, 1.0);
    float epsilon = 0.0001f;

    // Run the multiplicate update rules for max_iteration iterations
    for (int i = 0; i < max_iteration; i++) {
        // update H
        Eigen::MatrixXf WH = (W * H).array() + epsilon;
        Eigen::MatrixXf numerator = (V.array() / WH.array());
        numerator = numerator * H.transpose();
        Eigen::MatrixXf denominator = (H * ones).array() + epsilon;

        W = W.array() * numerator.array();
        W = W.array() / denominator.transpose().array();
        // std::cout << W << std::endl;
        // update H
        WH = (W * H).array() + epsilon;
        numerator = (V.array() / WH.array());
        numerator = W.transpose() * numerator;
        denominator = (ones * W).array() + epsilon;

        H = H.array() * numerator.array();
        H = H.array() / denominator.transpose().array();

        Eigen::MatrixXf differ = V - W * H;
        Eigen::MatrixXf measure = differ.array().square();
        float loss = measure.sum();
        std::cout << "norm(V-WH) is : " << loss << '\n';
    }
}

#include <ctime>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

void non_negative_matrix_factorization(const Eigen::MatrixXf& V,
                                       Eigen::MatrixXf& H, Eigen::MatrixXf& W,
                                       int num_basis, int max_iteration);

int main() {
    Eigen::MatrixXf V =
        ((Eigen::MatrixXf::Random(5, 8) + Eigen::MatrixXf::Ones(5, 8)) *
         122.5f);

    Eigen::MatrixXf H;
    Eigen::MatrixXf W;

    int max_iteration = 100;
    int num_basis = 5;
    non_negative_matrix_factorization(V, W, H, max_iteration, num_basis);
    return 0;
}

void non_negative_matrix_factorization(const Eigen::MatrixXf& V,
                                       Eigen::MatrixXf& W, Eigen::MatrixXf& H,
                                       int max_iteration, int num_basis) {
    int dim_feature = V.rows();
    int num_feature = V.cols();

    // Create H matrix with randomized initial values
    W = Eigen::MatrixXf(dim_feature, num_basis);
    W.setRandom();
    W.array().abs();
    W = ((Eigen::MatrixXf::Random(dim_feature, num_basis) +
          Eigen::MatrixXf::Ones(dim_feature, num_basis)) *
         122.5f);
    // Create our W with randomiyed initial values
    H = Eigen::MatrixXf(num_basis, num_feature);
    H.setRandom();
    H.array().abs();
    H = ((Eigen::MatrixXf::Random(num_basis, num_feature) +
          Eigen::MatrixXf::Ones(num_basis, num_feature)) *
         122.5f);
    std::cout << W * H << std::endl;

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
        std::cout << "norm(V-WH) is : " << loss << '\n';
    }
    std::cout << "V - W*H = \n" << V - W * H << std::endl;
}

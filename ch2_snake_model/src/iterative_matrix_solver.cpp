#include <Eigen/Dense>
#include <chrono>
#include <iostream>

template <typename T>
void print_matrix(const T& matrix) {
    std::cout << "******Print Matrix*******" << '\n';
    std::cout << " Row : " << matrix.rows() << ". Col :  " << matrix.cols()
              << '\n';
    std::cout << matrix << std::endl;
}

int main() {
    // initialize a random matrix A and vector b to apply this algorithm
    Eigen::Matrix3f A;
    A << 5, -2, 3, -3, 9, 1, 2, -1, -7;

    Eigen::Vector3f b;
    b << -1, 2, 3;
    // get corresponding upper lower and diagonal matrix
    Eigen::Matrix3f U = A.triangularView<Eigen::StrictlyUpper>();
    U = -U;
    Eigen::Matrix3f L = A.triangularView<Eigen::StrictlyLower>();
    L = -L;
    Eigen::Matrix3f D = A.diagonal().asDiagonal();

    // initialize the parameter
    Eigen::Vector3f x;
    x << 10000, 10000, 10000;

    // solve it analytically
    auto start = std::chrono::steady_clock::now();

    Eigen::VectorXf x_analytical_solve = A.ldlt().solve(b);
    print_matrix(x_analytical_solve);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time for analytical solution is : "
              << 1000 * elapsed_seconds.count() << "ms\n";

    int max_iteration = 30;
    start = std::chrono::steady_clock::now();

    for (int i = 0; i < max_iteration; i++) {
        x = (D - L).inverse() * U * x + (D - L).inverse() * b;
        std::cout << "|Gauss Seidel method: |"
                  << "Square error is:" << (A * x - b).norm() << std::endl;
        //
        // print_matrix(x);
    }
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end - start;
    std::cout << "elapsed time for gaussian seidel solution is : "
              << 1000 * elapsed_seconds.count() << "ms\n";
    print_matrix(x);
    x << 10000, 10000, 10000;

    for (int i = 0; i < max_iteration; i++) {
        x = D.inverse() * (L + U) * x + D.inverse() * b;
        std::cout << "|Jacobi iterative method: |"
                  << "Square error is:" << (A * x - b).norm() << std::endl;
        //
        // print_matrix(x);
    }
    print_matrix(x);

    return 0;
}

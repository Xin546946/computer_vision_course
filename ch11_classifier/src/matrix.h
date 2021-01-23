#pragma once
#include <vector>

template <typename T>
class Matrix {
   public:
    Matrix(int rows, int cols) : rows_(rows), cols_(cols), matrix_(cols * rows) {
    }
    Matrix(int rows, int cols, T init) : rows_(rows), cols_(cols), matrix_(cols * rows, init) {
    }

    T& at(int r, int c) {
        return matrix_[r * cols_ + c];
    }

    const int rows_;
    const int cols_;

   private:
    std::vector<T> matrix_;
};
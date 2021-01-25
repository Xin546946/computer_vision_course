/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#pragma once
#include <vector>

/**
 * @brief class Matrix 2d container
 *
 * @tparam T
 */
template <typename T>
class Matrix {
   public:
    /**
     * @brief Construct a new Matrix object
     *
     * @param rows
     * @param cols
     */
    Matrix(int rows, int cols) : rows_(rows), cols_(cols), matrix_(cols * rows) {
    }

    /**
     * @brief Construct a new Matrix object
     *
     * @param rows
     * @param cols
     * @param init
     */
    Matrix(int rows, int cols, T init) : rows_(rows), cols_(cols), matrix_(cols * rows, init) {
    }

    /**
     * @brief at function
     *
     * @param r
     * @param c
     * @return T&
     */

    T& at(int r, int c) {
        return matrix_[r * cols_ + c];
    }

    const int rows_;
    const int cols_;

   private:
    std::vector<T> matrix_;
};
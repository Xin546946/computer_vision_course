#include "snake.h"
#include <cmath>
#include <iostream>

ParamSnake::ParamSnake(double alpha, double beta, double gamma,
                       double step_size)
    : alpha_(alpha), beta_(beta), gamma_(gamma), step_size_(step_size) {
}
/**
 * @brief Check if contour is valid
 *
 * @param max_x
 * @param max_y
 * @param radius
 * @param center
 * @return true
 * @return false
 */
bool check_valid(int max_x, int max_y, double radius, cv::Point2d center) {
    return radius < std::min(std::min(max_x - center.x, center.x),
                             std::min(max_y - center.y, center.y));
}

/**
 * @brief Construct a new Contour:: Contour object
 *
 * @param max_x : Boundary in x axis of the image
 * @param max_y : Boundary in y axis of the image
 * @param radius : Define a circle contour with a fixed radius
 * @param center : Define a circle contour with a center point
 * @param num_points : The number of contour points
 */
Contour::Contour(int max_x, int max_y, double radius, cv::Point2d center,
                 int num_points)
    : points_(cv::Mat::zeros(cv::Size(num_points, 2), CV_64F)) {
    bool is_valid = check_valid(max_x, max_y, radius, center);

    // check if the radius and center is valid w.r.t the image size
    if (!is_valid) {
        std::cerr << "Your Contour are out of boundary." << std::endl;
        std::exit(-1);
    }

    double angle = 2 * M_PI / num_points;

    for (int i = 0; i < num_points; i++) {
        points_.at<double>(i, 0) = center.x + radius * cos(-i * angle);
        points_.at<double>(i, 1) = center.y + radius * sin(-i * angle);
    }
}

int Contour::get_num_points() const {
    return num_points_;
}
cv::Mat Contour::get_points() {
    return points_;
}
/**
 * @brief Construct a new Snake:: Snake object
 *
 * @param gvf_x : GVF result for dufuse the contour w.r.t. external force in x
 * axis
 * @param gvf_y : GVF result for dufuse the contour w.r.t. external force in y
 * axis
 * @param contour : initialized contour
 * @param param_snake : relavent parameters for snake model
 */
Snake::Snake(cv::Mat gvf_x, cv::Mat gvf_y, Contour contour,
             ParamSnake param_snake)
    : GradientDescentBase(param_snake.step_size_),
      param_snake_(param_snake),
      contour_(contour),
      gvf_x_(gvf_x),
      gvf_y_(gvf_y),
      gvf_contour_(cv::Size(contour_.get_num_points(), 2), CV_64F) {
}
/**
 * @brief Overlodaded operator for [], that points_[i] return the i-th conotur
 * point
 *
 * @param i
 * @return cv::Point2d&
 */
cv::Point2d& Contour::operator[](int i) {
    return points_.at<cv::Point2d>(i);
}
/*************************************************************************/
/******************FUNCTIONS FOR SNAKE CLASS******************************/
/************************************************************************/
/**
 * @brief : circularly shifts the values in the array input by downshift and
 * right shift elements
 *
 * @param matrix : matrix need to be dealt with
 * @param down_shift : down shift coefficient
 * @param right_shift : right shift coefficient
 * @return cv::Mat
 */
cv::Mat circshift(cv::Mat matrix, int down_shift, int right_shift) {
    down_shift = ((down_shift % matrix.rows) + matrix.rows) % matrix.rows;
    right_shift = ((right_shift % matrix.cols) + matrix.cols) % matrix.cols;
    cv::Mat output = cv::Mat::zeros(matrix.rows, matrix.cols, matrix.type());

    for (int i = 0; i < matrix.rows; i++) {
        int new_row = (i + down_shift) % matrix.rows;
        for (int j = 0; j < matrix.cols; j++) {
            // int new_colum = (j + down_shift) % input.cols;
            int new_column = (j + right_shift) % matrix.cols;
            output.at<double>(new_row, new_column) = matrix.at<double>(i, j);
        }
    }
    return output;
}

void Snake::cal_internal_force_matrix() const {
    //  build A matrix using helper function circshift
    cv::Mat A(contour_.get_num_points(), contour_.get_num_points(), CV_64F);
    cv::Mat Id = cv::Mat::eye(contour_.get_num_points(),
                              contour_.get_num_points(), CV_64F);
    A = 2 * Id + circshift(-1 * Id, 0, 1);
    A = A + circshift(-1 * Id, 0, -1);
    // Building B matrix using helper function circshift
    cv::Mat B(contour_.get_num_points(), contour_.get_num_points(), CV_64F);
    B = 6 * Id + circshift(-4 * Id, 0, 1);
    B += circshift(Id, 0, 2);
    B += circshift(-4 * Id, 1, 0);
    B += circshift(Id, 0, -2);
    // Build internal force matrix w.r.t. the corresponding parameters
    internal_force_matrix_ =
        param_snake_.alpha_ * A - param_snake_.beta_ * B + Id;
}

void Snake::initialize() {
    // Already initialize in constructor.
}

void Snake::update() {
    // TODO Need to check if the contour within the image boundary
    for (int index = 0; index < get_contour().rows; index++) {
        gvf_contour_.at<double>(index) = gvf_x_.at<double>(
            round(contour_.get_points().at<cv::Point2d>(index).x),
            round(contour_.get_points().at<cv::Point2d>(index).y));
    }
    contour_.get_points() =
        internal_force_matrix_.mul(contour_.get_points()) + gvf_contour_;
}

void Snake::print_terminate_info() const {
    std::cout << "Snake iteration finished." << std::endl;
}

void min_max(cv::Mat x, cv::Mat y, std::vector<cv::Point2d> C, double num,
             cv::Mat u) {
    // int size_1, size_2;
    // size_1, size_2 = u.size();
    std::cout << "The size of u is: " << u.size() << std::endl;
    size_t size_1 = u.rows;
    size_t size_2 = u.cols;

    // double tmp_1, tmp_2;
    for (int i{0}; i < C.size(); i++) {
        x.at<double>(i) = x.at<double>(i) > num ? x.at<double>(i) : num;
        x.at<double>(i) = x.at<double>(i) < size_1 ? x.at<double>(i) : size_1;
        y.at<double>(i) = y.at<double>(i) > num ? y.at<double>(i) : num;
        y.at<double>(i) = y.at<double>(i) < size_2 ? y.at<double>(i) : size_2;
    }
}

cv::Mat Snake::get_contour() {
    return contour_.get_points();
}

double Snake::compute_energy() {
    return 0.1;
}

std::string Snake::return_drive_class_name() const {
    return "Snake";
}

void roll_back_state() {
    int a = 1;
}
void back_up_state() {
    int a = 1;
}
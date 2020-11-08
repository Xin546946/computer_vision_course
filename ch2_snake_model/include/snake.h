#pragma once
#include "gradient_descent_base.h"
#include <opencv2/core.hpp>
#include <vector>

// namespace snake
/**
 * @brief contour in the snake model
 *
 */
class Contour {
   public:
    /**
     * @brief Construct a new Contour object
     *
     * @param max_x : cols of the image
     * @param max_y : rows of the image
     * @param radius : radius of the contour
     * @param center : center of the contour
     * @param num_points : points on the contour
     */
    Contour(int max_x, int max_y, double radius, cv::Point2d center,
            int num_points);
    Contour(const Contour& contour);

    cv::Vec2d& operator[](int index);
    cv::Mat get_points() const;
    int get_num_points() const;
    void update(cv::Mat update_step);

   protected:
    cv::Mat points_;
};

struct ParamSnake {
    ParamSnake(double alpha, double beta, double gamma, double step_size);
    double alpha_;
    double beta_;
    double gamma_;
    double step_size_;
};

class Snake : public GradientDescentBase {
   public:
    Snake(cv::Mat original_img, cv::Mat gvf_x, cv::Mat gvf_y, Contour contour,
          ParamSnake param_snake);

    Contour get_contour() const;

   private:
    void initialize() override;
    void update() override;

    void cal_internal_force_matrix();

    std::string return_drive_class_name() const override;
    void roll_back_state() override;
    void back_up_state() override;
    void print_terminate_info() const override;
    double compute_energy() override;

    cv::Mat original_img_;
    cv::Mat internal_force_matrix_;
    ParamSnake param_snake_;
    Contour contour_;
    Contour last_contour_;
    cv::Mat gvf_x_;
    cv::Mat gvf_y_;
    cv::Mat gvf_contour_;
};

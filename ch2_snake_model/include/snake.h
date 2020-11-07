#include "gradient_descent_base.h"

#include <opencv2/core.hpp>
#include <vector>

// namespace snake
class Contour {
   public:
    // Many methods could be implemented here
    Contour(int max_x, int max_y, double radius, cv::Point2d center,
            int num_points);
    // Contour(int max_x, int max_y, std::vector<Point> points);

    cv::Point2d& operator[](int index);
    cv::Mat get_points();
    int get_num_points() const;

   protected:
    cv::Mat points_;
    int num_points_;
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
    Snake(cv::Mat gvf_x, cv::Mat gvf_y, Contour contour,
          ParamSnake param_snake);

   private:
    s void initialize() override;
    void update() override;

    double compute_energy() override;
    void cal_internal_force_matrix() const;
    void print_terminate_info() const override;
    cv::Mat get_contour();

    std::string return_drive_class_name() const override;
    void roll_back_state() override;
    void back_up_state() override;

    mutable cv::Mat internal_force_matrix_;
    ParamSnake param_snake_;
    Contour contour_;
    cv::Mat gvf_x_;
    cv::Mat gvf_y_;
    cv::Mat gvf_contour_;
    cv::Mat contour_first_dev_;
    cv::Mat contour_second_dev_;
    cv::Mat contour_forth_dev_;
};

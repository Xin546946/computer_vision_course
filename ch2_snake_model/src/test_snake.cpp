#include "display.h"
#include "gvf.h"
#include "snake.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    // preprocess the input image
    cv::GaussianBlur(img, img, cv::Size(3, 3), 3, 3);

    cv::Mat grad_original_x, grad_original_y;
    cv::Sobel(img, grad_original_x, CV_64F, 1, 0, 3);
    cv::Sobel(img, grad_original_y, CV_64F, 0, 1, 3);

    // run gvf at first to get the gradient vecotor flow
    double smooth_term = 1e8;
    double step_size = 5e-10;
    ParamGVF param_gvf(smooth_term, step_size);
    GVF gvf(grad_original_x, grad_original_y, param_gvf);

    int max_iteration_gvf = 1e3;  // 2500
    gvf.run(max_iteration_gvf);   // parameter: max_iteration
    std::vector<cv::Mat> gvf_result = gvf.get_result_gvf();

    display_gvf(gvf_result[0], gvf_result[1], 1, true);
    std::cout << gvf_result[0].type() << std::endl;

    // run snake
    int max_x = gvf_result[0].rows;
    int max_y = gvf_result[1].cols;
    double radius = std::min(max_x, max_y) / 2.1f;  // 4
    cv::Point2d center(max_x / 2.f, max_y / 2.f);
    int num_points = 200;  // 300

    Contour contour(max_x, max_y, radius, center, num_points);
    ParamSnake param_snake(0.001, 0.06, 1.0f, 0.25, true);  // 0.001 0.06

    Snake snake_model(img, gvf_result[0], gvf_result[1], contour, param_snake);
    snake_model.run(600);
    Contour result_contour = snake_model.get_contour();
    display_contour(img, result_contour, 0);
    // ParamSnake param_snake_add_balloon_force(0.001, 0.06, 0.25, 1.0f, false);
    // Snake snake_model_improve(img, gvf_result[0], gvf_result[1],
    // result_contour,
    //                           param_snake_add_balloon_force);
    // snake_model_improve.run(1e3);
    // result_contour = snake_model_improve.get_contour();
    // display_contour(img, result_contour, 0);
    return 0;
}
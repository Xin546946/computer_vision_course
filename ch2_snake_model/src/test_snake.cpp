#include "display.h"
#include "gvf.h"
#include "snake.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    // preprocess the input image
    /**    cv::GaussianBlur(img, img, cv::Size(3, 3), 3, 3);

        cv::Mat grad_original_x, grad_original_y;
        cv::Sobel(img, grad_original_x, CV_64F, 1, 0, 3);
        cv::Sobel(img, grad_original_y, CV_64F, 0, 1, 3);

        double smooth_term = 1e8;
        double step_size = 1e-10;
        ParamGVF param_gvf(smooth_term, step_size);  // TODO DELETE 21
        GVF gvf(grad_original_x, grad_original_y, param_gvf);

        int max_iteration_gvf = 1e4;
        gvf.run(max_iteration_gvf);  // parameter: max_iteration
        std::vector<cv::Mat> gvf_result = gvf.get_result_gvf();

        // display gvf result please set save as true if you want to save it
        bool save = true;
        display_gvf(gvf_result[0], gvf_result[1], 0, save); */

    std::vector<cv::Mat> gvf_result(2);
    gvf_result[0] = cv::imread("./images/test_data/star_gvf_x.png");
    gvf_result[1] = cv::imread("./images/test_data/star_gvf_y.png");

    // Initialize a contour
    int max_x = gvf_result[0].rows;
    int max_y = gvf_result[1].cols;
    double radius = std::min(max_x, max_y) / 3.f;
    cv::Point2d center(max_x / 2.f, max_y / 2.f);
    int num_points = 100;
    Contour contour(max_x, max_y, radius, center, num_points);
    ParamSnake param_snake(1, 2, 3, 1);
    Snake snake_model(gvf_result[0], gvf_result[1], contour, param_snake);
    snake_model.run(100);
    Contour result_contour = snake_model.get_contour();
    display_contour(img, result_contour, 0);

    return 0;
}
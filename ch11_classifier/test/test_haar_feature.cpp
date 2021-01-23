#include "haar_feature.h"
#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    Matrix<std::vector<double>> test(109, 10);
    std::cout << "test pass" << '\n';

    cv::Mat img = read_img(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat temp = read_img(argv[2], cv::IMREAD_GRAYSCALE);

    DetectionWindow detection_window(temp.cols, temp.rows);

    detection_window.add_haar_rect(0.200, 0.150, 0.618, 0.317, (cv::Mat_<int>(1, 3) << -1, 1, -1));
    detection_window.add_haar_rect(0.200, 0.330, 0.600, 0.233, (cv::Mat_<int>(2, 1) << 1, -1));
    detection_window.add_haar_rect(0.400, 0.567, 0.300, 0.133, (cv::Mat_<int>(2, 2) << 1, -1, -1, 1));
    detection_window.add_haar_rect(0.360, 0.720, 0.340, 0.15, (cv::Mat_<int>(2, 1) << 1, -1));
    // show all sub img for testing visualization
    // detection_window.show_all_sub_image(temp, 0, 0);

    // convert the img to integral img
    cv::Mat integral_img = make_integral_img(img);
    cv::imshow("Integral img", get_float_mat_vis_img(integral_img));
    cv::waitKey(0);
    cv::Mat integral_img_temp = make_integral_img(temp);

    // compute haar feature (ground truth) for each haar rect
    cv::Mat feature_ground_truth = compute_haar_feature_vector(detection_window, integral_img_temp, 0, 0);
    std::cout << feature_ground_truth << '\n';
    // compute haar feature on the img via haar rects
    Matrix<cv::Mat> feature_matrix = compute_haar_feature_matrix(detection_window, integral_img);
    double min_dist = std::numeric_limits<double>::max();
    cv::Point detection_ul;
    std::vector<cv::Point> multi_detection_ul;

    std::vector<std::pair<double, cv::Point>> score_position;
    for (int r = 0; r < feature_matrix.rows_; r++) {
        for (int c = 0; c < feature_matrix.cols_; c++) {
            double dist = cv::norm(feature_matrix.at(r, c) - feature_ground_truth, cv::NORM_L2SQR);
            score_position.emplace_back(dist, cv::Point(c, r));
        }
    }
    auto cmp = [](std::pair<double, cv::Point> lhs, std::pair<double, cv::Point> rhs) { return lhs.first < rhs.first; };
    std::sort(score_position.begin(), score_position.end(), cmp);
    cv::Mat vis_bbox;
    cv::cvtColor(img, img, CV_GRAY2BGR);
    for (int i = 0; i < 80; i++) {
        vis_bbox = draw_bounding_box_vis_image(img, score_position[i].second.x, score_position[i].second.y,
                                               detection_window.get_width(), detection_window.get_height());
    }

    // detection_center returns the center of bounding box with min_dist
    cv::imshow("detection result", vis_bbox);
    cv::waitKey(0);

    return 0;
}

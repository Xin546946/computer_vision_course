#include <array>
#include <opencv2/core.hpp>

struct CallbackItem {
    CallbackItem(cv::Mat img, std::string win_name, cv::Scalar color_scrible)
        : img_(img), win_name_(win_name), color_(color_scrible){};
    std::string win_name_;
    cv::Mat img_;
    std::vector<cv::Point> points_;
    cv::Scalar color_;
};

std::array<std::vector<cv::Point>, 2> drag_to_get_fore_and_background_scrible(
    cv::Mat img);
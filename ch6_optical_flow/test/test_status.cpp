#include "feature_points_manager.cpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void mark_status_with_angle(const std::vector<cv::Vec2f>& motion, std::vector<uchar>& status, float rate) {
    std::vector<float> angle_vec(motion.size());
    std::transform(motion.begin(), motion.end(), angle_vec.begin(),
                   [=](cv::Vec2f m) { return atan2(m[1], m[0]) * 180 * M_1_PI; });
    std::cout << "**********Angle Vec********" << std::endl;
    for (auto i : angle_vec) {
        std::cout << i << '\n';
    }
    std::cout << "****************************" << '\n';
    float mid = median(angle_vec);
    std::cout << "Median :" << mid << '\n';
    std::vector<float>::iterator it_angle = angle_vec.begin();
    std::replace_if(status.begin(), status.end(),
                    [&](uchar i) {
                        bool is_outlier = ((*it_angle < mid - rate) || (*it_angle > mid + rate));
                        it_angle++;
                        return is_outlier;
                    },
                    0);
}

int main(int argc, char** argv) {
    std::vector<cv::Vec2f> motion{cv::Vec2f(1.4, 1.0), cv::Vec2f(1.9, 1.0), cv::Vec2f(1000.0, 1.0), cv::Vec2f(1.0, 1.0),
                                  cv::Vec2f(9.0, 0.0)};
    std::vector<uchar> status{1, 1, 1, 1, 1};
    float rate = 5;
    mark_status_with_angle(motion, status, rate);
    for (auto i : status) {
        std::cout << int(i) << '\n';
    }

    return 0;
}
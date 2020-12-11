#include "opencv_utils.h"
#include "opencv2/imgproc.hpp"

std::vector<cv::Mat> record_webcam() {
    std::vector<cv::Mat> result;
    cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if (!cap.open(0)) return {};
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;  // end of video stream
        cv::imshow("this is you, smile! :)", frame);
        auto key = cv::waitKey(10);
        if (key == 27) {
            break;  // stop capturing by pressing ESC
        } else {
            result.push_back(frame);
        }
    }
    return result;
}

cv::Point2i template_matching(cv::Mat img, cv::Mat temp) {
    assert(!img.empty() && !temp.empty());
    cv::Mat result;
    cv::matchTemplate(img, temp, result, cv::TM_CCORR_NORMED);
    double val_min, val_max;
    cv::Point center_min, center_max;
    cv::minMaxLoc(result, &val_min, &val_max, &center_min, &center_max, cv::Mat());
    return center_max;
}

cv::Mat get_sub_image(cv::Mat image, int x, int y, int width, int height) {
    cv::Rect img_rect = cv::Rect(cv::Point(0, 0), image.size());
    cv::Rect roi = cv::Rect(cv::Point(x - width / 2, y - height / 2), cv::Size(width, height));
    cv::Rect intersection = img_rect & roi;

    cv::Mat sub_img = cv::Mat::zeros(intersection.size(), image.type());
    image(intersection).copyTo(sub_img);
    return sub_img;
}
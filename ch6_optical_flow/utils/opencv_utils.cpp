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

cv::Rect get_intersection(cv::Mat image, int x, int y, int width, int height) {
    cv::Rect img_rect = cv::Rect(cv::Point(0, 0), image.size());
    cv::Rect roi = cv::Rect(cv::Point(x - width / 2, y - height / 2), cv::Size(width, height));
    cv::Rect intersection = img_rect & roi;
    return intersection;
}

cv::Mat get_sub_image_around(cv::Mat image, int x, int y, int width, int height) {
    cv::Rect intersection = get_intersection(image, x, y, width, height);
    cv::Mat sub_img = cv::Mat::zeros(intersection.size(), image.type());
    image(intersection).copyTo(sub_img);
    return sub_img;
}

cv::Mat get_bounding_box_vis_image(cv::Mat image, int x, int y, int width, int height) {
    cv::Mat vis_bbox = image.clone();
    assert(image.type() == CV_8UC1);

    cv::cvtColor(vis_bbox, vis_bbox, CV_GRAY2BGR);
    cv::rectangle(vis_bbox, cv::Rect2i(x, y, width, height), cv::Scalar(0, 0, 255));
    return vis_bbox;
}

template <typename T>
void put_val_from_ul(T val, cv::Mat input_mat, int x_ul, int y_ul, int width, int height) {
    cv::Rect intersection = get_intersection(image, x_ul, y_ul, width, height);
    input_mat(intersection) = val * cv::Mat::ones(intersection.size(), CV_8UC1);
}

template <typename T>
void put_val_around(T val, cv::Mat input_mat, int x_center, int y_center, int width, int height) {
    assert(width % 2 == 1 && height % 2 == 1);
    put_val_from_ul(val, input_mat, x_ul - width / 2, y_ul - height / 2, width, height);
}
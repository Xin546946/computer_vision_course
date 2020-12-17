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

cv::Rect get_intersection_from_ul(cv::Mat image, int x, int y, int width, int height) {
    cv::Rect img_rect = cv::Rect(cv::Point(0, 0), image.size());
    cv::Rect roi = cv::Rect(cv::Point(x, y), cv::Size(width, height));
    cv::Rect intersection = img_rect & roi;
    return intersection;
}

cv::Rect get_intersection_around(cv::Mat image, int x, int y, int width, int height) {
    return get_intersection_from_ul(image, x - width / 2, y - height / 2, width, height);
}

cv::Mat get_sub_image_around(cv::Mat image, int x, int y, int width, int height) {
    cv::Rect intersection = get_intersection_around(image, x, y, width, height);
    cv::Mat sub_img = cv::Mat::zeros(intersection.size(), image.type());
    image(intersection).copyTo(sub_img);
    return sub_img;
}

cv::Mat draw_bounding_box_vis_image(cv::Mat image, float x, float y, float width, float height) {
    cv::rectangle(image, cv::Rect2i(x, y, width, height), cv::Scalar(0, 255, 0), 2);
    return image;
}

/**
 * @brief
 *
 * @param input
 * @param flag  = 0 x , = 1 y
 * @return cv::Mat
 */
cv::Mat do_sobel(cv::Mat input, int flag) {
    cv::Mat im = input.clone();
    if (im.channels() != 1) {
        cv::cvtColor(input, im, CV_BGR2GRAY);
    }
    if (im.type() != CV_64FC1) {
        im.convertTo(im, CV_64FC1);
    }

    cv::Mat output(im.size(), im.type());
    for (int r = 0; r < im.rows; r++) {
        for (int c = 0; c < im.cols; c++) {
            int r_dhs = r + flag;
            int c_rhs = c + (1 - flag);
            c_rhs = std::min(std::max(0, c_rhs), im.cols - 1);
            r_dhs = std::min(std::max(0, r_dhs), im.rows - 1);
            int r_uhs = r - flag;
            int c_lhs = c - (1 - flag);
            c_lhs = std::min(std::max(0, c_lhs), im.cols - 1);
            r_uhs = std::min(std::max(0, r_uhs), im.rows - 1);
            output.at<double>(r, c) = 0.5 * (im.at<double>(r_dhs, c_rhs) - im.at<double>(r_uhs, c_lhs));
        }
    }
    return output;
}
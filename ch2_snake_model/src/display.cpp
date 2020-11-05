
#include "display.h"

void draw_optical_flow(cv::Mat& fx, cv::Mat& fy, cv::Mat& cflowmap, int step,
                       double scaleFactor, cv::Scalar& color) {
    for (int r = 0; r < cflowmap.rows; r += step)
        for (int c = 0; c < cflowmap.cols; c += step) {
            cv::Point2f fxy;

            fxy.x = fx.at<float>(r, c);
            fxy.y = fy.at<float>(r, c);

            if (fxy.x != 0 || fxy.y != 0) {
                cv::line(cflowmap, cv::Point(c, r),
                         cv::Point(cvRound(c + (fxy.x) * scaleFactor),
                                   cvRound(r + (fxy.y) * scaleFactor)),
                         color, 1, cv::LINE_AA);
            }
            cv::circle(cflowmap, cv::Point(c, r), 1, cv::Scalar(255, 0, 0), 1);
        }
}

void display_gvf(cv::Mat fx, cv::Mat fy, int delay) {
    cv::Mat cflowmap = cv::Mat::zeros(fx.size(), CV_8UC3);

    int step = 8;
    double scaleFactor = 7;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    cv::Mat disp_fx = fx.clone();
    cv::Mat disp_fy = fy.clone();
    cv::normalize(disp_fx, disp_fx, -1, 1, cv::NORM_MINMAX);
    cv::normalize(disp_fy, disp_fy, -1, 1, cv::NORM_MINMAX);
    draw_optical_flow(disp_fx, disp_fy, cflowmap, step, scaleFactor, color);
    disp_image(cflowmap, "gvf display", delay);
}

//--Overloaded functions to display an image in a new window--//
void disp_image(cv::Mat& img) {
    if (img.empty()) {  // Read image and display after checking for image
                        // validity
        std::cout << "Error reading image file!";
        std::cin.ignore();
    } else {
        cv::namedWindow("Image", 0);
        cv::imshow("Image", img);
        cv::waitKey();
    }
}

void disp_image(cv::Mat& img, cv::String windowName) {
    if (img.empty()) {  // Read image and display after checking for image
                        // validity
        std::cout << "Error reading image File!";
        std::cin.ignore();
    } else {
        cv::imshow(windowName, img);
        cv::waitKey();
    }
}

void disp_image(cv::Mat& img, cv::String windowName, int delay) {
    if (img.empty()) {  // Read image and display after checking for image
                        // validity
        std::cout << "Error reading image File!";
        std::cin.ignore();
    } else {
        cv::namedWindow(windowName, 0);
        cv::imshow(windowName, img);
        cv::waitKey(delay);
    }
}

void disp_image(cv::Mat& img, cv::String windowName, cv::String error_msg) {
    if (img.empty()) {  // Read image and display after checking for image
                        // validity
        std::cout << error_msg;
        std::cin.ignore();
    } else {
        cv::namedWindow(windowName, 0);
        cv::imshow(windowName, img);
        cv::waitKey();
    }
}

void disp_image(cv::Mat& img, cv::String windowName, cv::String error_msg,
                int delay) {
    if (img.empty()) {  // Read image and display after checking for image
                        // validity
        std::cout << error_msg;
        std::cin.ignore();
    } else {
        cv::namedWindow(windowName, 0);
        cv::imshow(windowName, img);
        cv::waitKey(delay);
    }
}

#include "display.h"

void drawOptFlowMap(cv::Mat& fx, cv::Mat& fy, cv::Mat& cflowmap, int step,
                    double scaleFactor, cv::Scalar& color) {
    for (int r = 0; r < cflowmap.rows; r += step)
        for (int c = 0; c < cflowmap.cols; c += step) {
            cv::Point2f fxy;

            fxy.x = fx.at<float>(r, c);
            fxy.y = fy.at<float>(r, c);

            if (fxy.x != 0 || fxy.y != 0) {
                cv::line(cflowmap, cv::Point(r, c),
                         cv::Point(cvRound(r + (fxy.x) * scaleFactor),
                                   cvRound(c + (fxy.y) * scaleFactor)),
                         color);
            }
            cv::circle(cflowmap, cv::Point(r, c), 1, color, -1);
        }
}

//--Overloaded functions to display an image in a new window--//
void dispImage(cv::Mat& img) {
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

void dispImage(cv::Mat& img, cv::String windowName) {
    if (img.empty()) {  // Read image and display after checking for image
                        // validity
        std::cout << "Error reading image File!";
        std::cin.ignore();
    } else {
        cv::imshow(windowName, img);
        cv::waitKey();
    }
}

void dispImage(cv::Mat& img, cv::String windowName, int delay) {
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

void dispImage(cv::Mat& img, cv::String windowName, cv::String error_msg) {
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

void dispImage(cv::Mat& img, cv::String windowName, cv::String error_msg,
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

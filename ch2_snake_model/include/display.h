#ifndef IMUTILS_H
#define IMUTILS_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

void dispImage(cv::Mat& img, cv::String windowName);
void dispImage(cv::Mat& img, cv::String windowName, int delay);
void drawOptFlowMap(cv::Mat& fx, cv::Mat& fy, cv::Mat& cflowmap, int step,
                    double scaleFactor, cv::Scalar& color);

#endif

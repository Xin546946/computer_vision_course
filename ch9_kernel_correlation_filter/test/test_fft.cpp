#include "opencv_utils.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// compute an fft for an img
// get amplitude
// compute ifft img from fourier img

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    // img.convertTo(img, CV_64FC1);
    int rows_dft = cv::getOptimalDFTSize(img.rows);
    int cols_dft = cv::getOptimalDFTSize(img.cols);
    cv::Mat padded;  // expand input image to optimal size
    cv::copyMakeBorder(img, padded, 0, rows_dft - img.rows, 0, cols_dft - img.cols, cv::BORDER_CONSTANT,
                       cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32FC1)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);  // Add to the expanded another plane with zerosS
    cv::dft(complexI, complexI);     // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    cv::split(complexI, planes);                     // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    cv::magnitude(planes[0], planes[1], planes[0]);  // planes[0] = magnitude
    cv::Mat magI = planes[0];
    magI += cv::Scalar::all(1);  // switch to logarithmic scale
    log(magI, magI);
    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(cv::Rect(0, 0, magI.cols & -2, magI.rows & -2));
    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    // Shift the origin center of Fourier img
    int cx = magI.cols / 2;
    int cy = magI.rows / 2;
    cv::Mat q0(magI, cv::Rect(0, 0, cx, cy));    // Top-Left - Create a ROI per quadrant
    cv::Mat q2(magI, cv::Rect(0, cy, cx, cy));   // Bottom-Left
    cv::Mat q1(magI, cv::Rect(cx, 0, cx, cy));   // Top-Right
    cv::Mat q3(magI, cv::Rect(cx, cy, cx, cy));  // Bottom-Right
    cv::Mat tmp;                                 // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);  // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);
    cv::normalize(magI, magI, 0, 1, cv::NORM_MINMAX);  // Transform the matrix with float values into a
                                                       // viewable image form (float between values 0 and 1).
    cv::imshow("Input Image", img);                    // Show the result
    cv::imshow("spectrum magnitude", magI);
    cv::waitKey(0);

    // calculating the idft
    cv::Mat inverseTransform;
    cv::dft(complexI, inverseTransform, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    cv::normalize(inverseTransform, inverseTransform, 0, 1, CV_MINMAX);
    cv::imshow("Reconstructed", inverseTransform);
    cv::waitKey();
    return 0;
}
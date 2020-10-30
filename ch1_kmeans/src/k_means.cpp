#include "k_means.h"

Kmeans::Kmeans(cv::Mat img, const int k){
    centers_.resize(k);
    samples_.resize(img.rows * img.cols);

    for(int r = 0; r < img.rows; r++) {
        for (int c = 0; c < img.cols; c++) {
            std::array<float, 3> tmp;
            for (int channel = 0; channel < 3; channel++) {
                tmp[channel] = img.at<cv::Vec3f>(r,c)[channel]; 
                
            }
            
        }
    }
}
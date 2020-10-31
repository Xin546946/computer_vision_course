#include "k_means.cpp"
// #include "k_means.h"
// #include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    if (img.empty()) {
        std::cerr << "useage: ./test_k_means input_image_path k iteration\n "
                     "example: ./test_k_means "
                     "../images/test_data/lena.png 3 10"
                  << std::endl;
        std::exit(-1);
    }

    if (img.channels() != 3) {
        std::cout << "please use a image with 3 channels";
        std::exit(-1);
    }

    int k = strtol(argv[2], NULL, 10);
    int iteration = strtol(argv[3], NULL, 10);

    int convergence_radius = 1e-6;

    Kmeans kmeans(img, k);
    kmeans.run(iteration, convergence_radius);

    std::vector<Sample> samples = kmeans.get_result_samples();
    std::vector<Center> centers = kmeans.get_result_centers();

    cv::Mat result(img.size(), img.type());

    for (const Sample& sample : samples) {
        for (int channel = 0; channel < 3; channel++) {
            result.at<cv::Vec3b>(sample.row_, sample.col_)[channel] =
                centers[sample.label_].feature_[channel];
        }
    }
    cv::Mat concat_img;
    cv::hconcat(img, result, concat_img);
    cv::imshow("left: original image, right: kmeans result", concat_img);
    cv::waitKey(0);

    // figure out the relationship of k and function value
    // TODO Value function is not always decreated!
    std::cout << "k : value function : difference" << '\n';
    float last_value_function = 0;
    for (k = 2; k < 20; k++) {
        Kmeans kmeans_property(img, k);
        kmeans_property.run(iteration, convergence_radius);
        std::vector<Sample> samples_property =
            kmeans_property.get_result_samples();
        std::vector<Center> centers_property =
            kmeans_property.get_result_centers();
        float value_function = 0;
        float differ__rate;
        for (Sample sample : samples_property) {
            value_function += calc_square_distance(
                sample.feature_, centers_property[sample.label_].feature_);
        }
        if (last_value_function != 0)
            differ__rate = (last_value_function - value_function);
        last_value_function = value_function;
        if (last_value_function != 0)
            std::cout << k << " : " << value_function << " : " << differ__rate
                      << '\n';
    }

    return 0;
}
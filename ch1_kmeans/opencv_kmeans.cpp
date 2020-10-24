// This program is writen for using k means algorithm to segment images.
#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

/**
 * @brief  A function to clurster a image, based on opencv k-means function,
 * step1: prepare the data according to opencv api
 * step2: call opencv k-means
 * step3: return a image to visualize the result
 *
 * @param input: input image with 3 channels to be clustered by color
 * @param K: Number of clusters to split the image by
 * @return result: image to visualize the clustered result.
 */
Mat k_means(Mat Input, int k);

int main(int argc, char* argv[]) {
    // read the image and check if the image is read successfully.
    Mat input_image = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    if (input_image.empty()) {
        cerr << "useage: ./opencv_func input_image_path output_image_path \n "
                "example: ./opencv_func "
                "../images/test_data/lena.png "
                "../images/test_result/lena_result_opencv.png"
             << endl;
        exit(-1);
    }

    if (input_image.channels() != 3) {
        cerr << "please read an image with 3 channels !";
    }

    Mat clustered_image = k_means(input_image, 2);

    if (argv[2] != nullptr) {
        imwrite(argv[2], clustered_image);
    }
    imshow("Clustered image", clustered_image);
    waitKey(0);

    return 0;
}

Mat k_means(Mat input, int k) {
    // step1: prepare the data according to opencv api
    cout << "input size: " << input.size() << '\n';
    cout << "input channels: " << input.channels() << '\n';

    // opencv k-means need float data type as input
    Mat input_float;
    input.convertTo(input_float, CV_32FC3);

    int num_feature = input.cols * input.rows;

    Mat features;  // each row of features is a feature vector

    // trick of opencv to avoid copy: use reshape to make a new header
    features = input_float.reshape(1, num_feature);

    cout << "feature dimension : " << features.cols << '\n';

    cout << "feature num : " << features.rows << '\n';

    // step2: call opencv k-means

    Mat labels, centers;

    auto startTime = std::chrono::steady_clock::now();
    kmeans(features, k, labels,
           TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1, 1.0), 10,
           KMEANS_PP_CENTERS, centers);
    auto endTime = std::chrono::steady_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    cout << "k-means using opencv function costs: " << elapsedTime.count()
         << " milliseconds" << endl;

    // step3: return a image to visualize the result
    Mat result(input.size(), input.type());

    int num_channel = input.channels();
    for (int r = 0; r < input.rows; ++r) {
        for (int c = 0; c < input.cols; ++c) {
            int cluster_idx = labels.at<int>(c + r * input.cols);
            result.at<Vec3b>(r, c) = centers.at<Vec3f>(cluster_idx);
        }
    }
    return result;
}
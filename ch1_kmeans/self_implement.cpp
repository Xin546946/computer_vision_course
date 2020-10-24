#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <set>
#include <string>
#include <vector>
using namespace std;
using namespace cv;

static random_device rd;
static mt19937 rng(rd());

/**
 * @brief return a vector, contain n random number in the range [0, max_idx];
 *
 * @param n: num of wanted random index
 * @return set<int>: result random index set
 */
set<int> get_random_index(int max_idx, int n) {
    std::uniform_int_distribution<int> dist(1, max_idx + 1);

    set<int> random_idx;
    while (random_idx.size() < n) {
        random_idx.insert(dist(rng) - 1);
    }
    return random_idx;
}
/************************************************************************************************************
 * initialize the centers in the k means clustering
 * method 1: choose the initial centers in the k means clustering in a random
 *manner method 2: Chooses the initial centers in the k-means using Gonzales'
 *algorithm. so that the centers are spaced apart from each other. Method 3:
 *Chooses the initial centers in the k-means using the algorithm proposed in the
 *KMeans++ paper:
 **************************************************************************************************************/

// Method 1
void initial_random_center_1(Mat features, vector<Vec3f>& centers) {
    int k = centers.size();
    set<int> random_idx = get_random_index(features.rows - 1, centers.size());
    int i = 0;
    for (auto it = random_idx.begin(); it != random_idx.end(); ++it, i++) {
        centers[i] = features.row(*it);
    }
}

// Method 2
/* void initial_random_center_2(Mat features, vector<Vec3f>& center_pixel) {
    random_number_generator(image_size, random_index);
    center_pixel.push_back(samples.at<Vec3b>(random_index.at(0)));
    // cout << "center_pixel[0]" << center_pixel.at(0)<< endl;

    double distance{};
    double tmp_distance{};

    for (int index{1}; index < k; ++index) {
        int best_index = -1;
        int best_val = 0;
        for (int i{0}; i < image_size; ++i) {
            distance = norm(samples.at<Vec3b>(i), center_pixel.at(0), CV_L2);
            for (int j{1}; j < index; ++j) {
                tmp_distance = norm(samples.at<Vec3b>(i), center_pixel.at(j),
CV_L2); if (tmp_distance < distance) { distance = tmp_distance;
                }
            }

            if (distance > best_val) {
                best_val = distance;
                best_index = i;
            }
        }
        if (best_index != -1) {
            center_pixel.push_back(samples.at<Vec3b>(best_index));
        } else {
            break;
        }
    }
}

// Method 3
void initial_random_center_3(Mat features, vector<Vec3f>& center_pixel) {
    // cout << "Start initialization" << endl;
    vector<int> random_index;
    random_number_generator(image_size, random_index);
    // choose the first center pixel randomly.
    center_pixel.push_back(samples.at<Vec3b>(random_index.at(0)));

    // calculate the distance between the first center pixel and each pixel(rgb
or gray value) in the image. int current_distance_sum = 0; vector<int>
distance_vector{}; int current_distance{}; for (int i = 0; i < image_size; i++)
{ current_distance = norm(samples.at<Vec3b>(i), center_pixel.at(0), NORM_L2SQR);
        distance_vector.push_back(current_distance);
        current_distance_sum += current_distance;
        // cout << 1<< endl;
    }
    // cout << "test4: " << endl;
    int i{0};
    int center_count{0};
    for (center_count = 1; center_count < k; center_count++) {
        int best_new_distance_sum = -1;
        int best_new_index = -1;
        vector<int> random_distance_vec{};
        int random_distance{};
        // cout <<"current_distance_sum" <<  current_distance_sum<< endl;
        // random_number_generator(current_distance_sum,random_distance_vec);
        // random_distance = random_distance_vec.at(0);
        srand(time(NULL));
        random_distance = 1 + (rand() % current_distance_sum);
        // cout << "Go into for loop " << endl;
        for (i; i < image_size - 1; ++i) {
            if (random_distance <= distance_vector.at(i))
                break;
            else
                random_distance -= distance_vector.at(i);
            // cout << "test4: "<< random_distance << endl;
        }
        // cout << "test3: " << i << endl;

        // Compute the new potential
        int new_distance_sum = 0;
        for (int index = 0; index < image_size; index++) {
            current_distance = norm(samples.at<Vec3b>(index),
samples.at<Vec3b>(i), NORM_L2SQR); new_distance_sum +=
std::min(current_distance, distance_vector.at(index));
        }

        // Store the best result
        if ((best_new_index < 0) || (new_distance_sum < best_new_index)) {
            best_new_distance_sum = new_distance_sum;
            best_new_index = i;
        }

        // Add the appropriate center
        center_pixel.push_back(samples.at<Vec3b>(best_new_index));
        current_distance_sum = best_new_distance_sum;
        for (int index = 0; index < image_size; index++) {
            current_distance = norm(samples.at<Vec3b>(index),
samples.at<Vec3b>(i), NORM_L2SQR); new_distance_sum +=
std::min(current_distance, distance_vector.at(index));
        }
    }
    // cout << "Finish initialization" << endl;
} */

// check the similarity of image and clustered image
double check_similarity(Mat img1, Mat img2) {
    if (img1.rows != img2.rows && img1.cols != img2.cols) {
        cout << "Please give the images with the same size! " << endl;
        return -1;
    }
    // cout << img1.at<Vec3b>(1,2)<< endl;
    // cout << static_cast<double>(img1.at<Vec3b>(1,1)[5]) << endl;
    double similarity1, similarity2, similarity3{0};
    int M1, M2, M3;
    int K1, K2, K3;
    M1 = M2 = M3 = K1 = K2 = K3 = 0;

    for (int y = 0; y < img1.rows; ++y) {
        for (int x = 0; x < img1.cols; ++x) {
            similarity1 += pow((static_cast<double>(img1.at<Vec3b>(y, x)[0]) -
                                static_cast<double>(img2.at<Vec3b>(y, x)[0])),
                               2);
            M1 += pow(static_cast<double>(img1.at<Vec3b>(y, x)[0]), 2);
            K1 += pow(static_cast<double>(img2.at<Vec3b>(y, x)[0]), 2);
            similarity2 += pow((static_cast<double>(img1.at<Vec3b>(y, x)[1]) -
                                static_cast<double>(img2.at<Vec3b>(y, x)[1])),
                               2);
            M2 += pow(static_cast<double>(img1.at<Vec3b>(y, x)[1]), 2);
            K2 += pow(static_cast<double>(img2.at<Vec3b>(y, x)[1]), 2);
            similarity3 += pow((static_cast<double>(img1.at<Vec3b>(y, x)[2]) -
                                static_cast<double>(img2.at<Vec3b>(y, x)[2])),
                               2);
            M3 += pow(static_cast<double>(img1.at<Vec3b>(y, x)[2]), 2);
            K3 += pow(static_cast<double>(img2.at<Vec3b>(y, x)[2]), 2);
        }
    }
    /*
    cout << "Similarity1: " << similarity1 <<  endl;
    cout << "Similarity2: " << similarity2 <<  endl;
    cout << "Similarity3: " << similarity3 <<  endl;
    cout << K1 << endl << M1 << endl << K2 << endl << M2 << endl;
    cout << static_cast<double>((sqrt(M2)*sqrt(K2))) << endl;
    cout << static_cast<double>((sqrt(M3)*sqrt(K3))) << endl;
    */
    double similarity{0};
    similarity = similarity1 / static_cast<double>(sqrt(M1) * sqrt(K1));
    similarity += similarity2 / static_cast<double>(sqrt(M2) * sqrt(K2));
    similarity += similarity3 / static_cast<double>(sqrt(M3) * sqrt(K3));
    similarity = 1 - similarity / 3;
    // cout << "The similarity of two images is: " << similarity << endl;
    return similarity;
}

// check convergencetest
float check_convergence(const vector<Vec3f>& centers_cur,
                        const vector<Vec3f>& centers_last) {
    float convergence_rate = 0.0f;

    for (int i = 0; i < centers_cur.size(); i++) {
        convergence_rate += norm(centers_cur[i], centers_last[i], CV_L2);
    }
    return convergence_rate;
}

// kmeans function
void kmeans(Mat img, int num_cluster, int max_iteration,
            double smallest_convergence_rate, string init_method = "method1") {
    Mat img_float;
    img.convertTo(img_float, CV_32FC3);

    int num_feature = img.cols * img.rows;
    int dim_feature = img.channels();
    // trick of opencv to avoid copy: use reshape to make a new header
    Mat features = img_float.reshape(
        1, num_feature);  // each row of features is a feature vector

    auto startTime = std::chrono::steady_clock::now();
    vector<Vec3f> centers_cur(num_cluster);

    if (init_method == "method1")
        initial_random_center_1(features, centers_cur);
    /*     else if (init_method == "method2")
            initial_random_center_2(features, centers_cur);
        else if (init_method == "method3")
            initial_random_center_3(features, centers_cur); */
    else
        std::cout << "Please enter the valid method" << endl;

    // test the center pixel
    int it = 0;
    vector<int> cluster_idx_per_feature(num_feature);
    vector<Vec3f> centers_last(centers_cur);
    while (it < max_iteration) {
        it++;

        vector<int> num_feature_per_cluster(num_cluster, 0);
        vector<Vec3f> sum_per_cluster(num_cluster, Vec3f(0.0f, 0.0f, 0.0f));

        for (int i_feature = 0; i_feature < features.rows; ++i_feature) {
            Vec3f feature = features.row(i_feature);

            int i_cluster_with_min_dist = 0;
            float min_dist = std::numeric_limits<float>::max();
            for (int i_cluster = 0; i_cluster < num_cluster; ++i_cluster) {
                float dist = norm(centers_cur[i_cluster], feature, CV_L2);
                if (dist < min_dist) {
                    min_dist = dist;
                    i_cluster_with_min_dist = i_cluster;
                }
            }

            num_feature_per_cluster[i_cluster_with_min_dist]++;
            cluster_idx_per_feature[i_feature] = i_cluster_with_min_dist;
            sum_per_cluster[i_cluster_with_min_dist] += feature;
        }

        for (int i_cluster = 0; i_cluster < num_cluster; ++i_cluster) {
            centers_cur[i_cluster] =
                sum_per_cluster[i_cluster] /
                (num_feature_per_cluster[i_cluster] + 0.001f);
        }

        float convergence_rate = check_convergence(centers_cur, centers_last);
        std::cout << "The convergence rate is: " << convergence_rate << '\n';
        if (convergence_rate < smallest_convergence_rate) {
            break;
        }
        centers_last = centers_cur;
    }

    if (it < max_iteration)
        std::cout << "The number of iteration is: " << it
                  << " until it convergence\n";
    else
        std::cout << "It did not convergence if the number of iteration is: "
                  << it << '\n';

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    cout << "Kmeans Using self implementation costs: " << elapsedTime.count()
         << " milliseconds\n";

    Mat result(img.size(), img.type());

    for (int r = 0; r < result.rows; ++r) {
        for (int c = 0; c < result.cols; ++c) {
            int cluster_id = cluster_idx_per_feature[r * result.cols + c];
            result.at<Vec3b>(r, c) =
                static_cast<Vec3b>(centers_cur[cluster_id]);
        }
    }

    Mat img_result_concate;
    hconcat(img, result, img_result_concate);
    imshow("original image(left) and clustered image(right)",
           img_result_concate);
    waitKey(0);

    imwrite(init_method + ".png", result);
    double similarity = check_similarity(img, result);
    cout << "The similarity is: " << similarity << endl;
}

int main(int argc, char* argv[]) {
    for (int i = 0; i < argc; ++i) cout << argv[i] << endl;

    Mat input_img = imread(argv[1]);

    int num_clusters = strtol(argv[2], NULL, 10);
    int iteration = strtol(argv[3], NULL, 10);
    double smallest_convergence_rate = 0.1;  // strtol(argv[4], NULL, 10);
    string init_method = argv[4];
    kmeans(input_img, num_clusters, iteration, smallest_convergence_rate,
           init_method);
    return 0;
}
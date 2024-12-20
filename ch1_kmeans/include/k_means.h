#include <array>
#include <opencv2/core.hpp>
#include <random>
#include <set>

struct Sehouette_Coefficient {
    Sehouette_Coefficient(float a = 0, float b = 0) : a_(a), b_(b){};
    float a_;
    float b_;
};
/**
 * @brief: Each sample is the data that to be clustered
 *
 */
struct Sample {
    Sample(const std::array<float, 3>& feature, const int row, const int col,
           const int label = -1)
        : feature_(feature), row_(row), col_(col), label_(label){};

    std::array<float, 3> feature_;  // feature vector of the data

    int label_;  // corresponding center id
    int row_;    // row in original image
    int col_;    // col in original image
};

struct Center {
    std::array<float, 3> feature_;  // center's position
};

/**
 * @brief K means class
 *
 */
class Kmeans {
   public:
    Kmeans(cv::Mat img, const int k);

    std::vector<Sample> get_result_samples() const;
    std::vector<Center> get_result_centers() const;
    // int get_k() const;
    // std::vector<Center> get_initial_center_for_test();
    void run(int max_iteration, float smallest_convergence_rate);
    void test_value_function(cv::Mat img);
    void sehouette_coefficient(cv::Mat img);
    Sehouette_Coefficient get_sehouette_coefficient(Sample sample);

   private:
    void initialize_centers();
    void update_centers();
    void update_labels();
    bool is_terminate(int current_iter, int max_iteration,
                      float smallest_convergence_rate) const;

    std::vector<Sample> samples_;       // all the data to be clustered
    std::vector<Center> centers_;       // center of each cluster
    std::vector<Center> last_centers_;  // center of last iteration
};

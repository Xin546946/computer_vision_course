#include <array>
#include <opencv2/core.hpp>
#include <random>

struct Sample {
    Sample(const std::array<float, 3>& feature, const int row, const int col,
           const int label = -1)
        : feature_(feature), row_(row), col_(col), label_(label){};

    std::array<float, 3> feature_;
    int label_;
    int row_;
    int col_;
};

struct Center {
    std::array<float, 3> position_;
};

class Kmeans {
   public:
    Kmeans(cv::Mat img, const int k);

    std::vector<Sample> get_result() const;
    void run(int max_iteration, float smallest_convergence_rate);

   private:
    void initial_centers();
    void update_centers();
    void update_labels();

    bool is_terminate(int current_iter, int max_iteration,
                      float smallest_convergence_rate) const;

    std::set<int> get_random_index(int max_idx, int n) const;

    std::vector<Sample> samples_;
    std::vector<Center> centers_;
    std::vector<Center> last_centers_;

    std::random_device rd;
    std::mt19937 rng;
};

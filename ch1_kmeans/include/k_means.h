#include <opencv2/core.hpp>
#include <vector>

struct Sample {
    Sample(const std::array<float, 3>& feature, const int row, const int col,
           const int label_ = -1);

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
    Kmeans(cv::Mat samples, const int k);
    void run();

   private:
    void initial_center();
    void update_center();
    void update_label();

    std::vector<Sample> samples_;
    std::vector<Center> centers_;
}
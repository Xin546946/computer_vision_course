#include <opencv2/core.hpp>
#include <vector>

struct Sample {
    std::vector<int> feature_;
    int label_;
    int row_;
    int col_;
}

class Kmeans {
   public:
    Kmeans(cv::Mat samples, const int k);
    void run();

   private:
    void initial_center();
    void update_center();
    void update_label();

    cv::Mat samples_;
    std::vector<Sample> centers_;
}
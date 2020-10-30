#include <opencv2/core.hpp>
#include <vector>

struct Sample {
    std::vector<int> feature;

    int label;
    int r;
    int c;
}

class Kmeans {
   public:
    Kmeans(cv::Mat samples, const int k);
    void run();

   private:
    void initial_center();
    void update_center();
    void update_label();

    cv::Mat m_samples;
    std::vector<Sample> centers;
}
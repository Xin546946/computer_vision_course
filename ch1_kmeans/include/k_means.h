#include <array>
#include <opencv2/core.hpp>

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
    void run();

   private:
    void initial_center(){
        
    }
    void update_center();
    void update_label();

    std::vector<Sample> samples_;
    std::vector<Center> centers_;
};
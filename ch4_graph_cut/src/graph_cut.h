#include "image_graph.h"
#include "interaction_tool.h"
#include <stack>

enum class SegType { FOREGROUND = 1, BACKGOUND = 2 };
class GraphCut {
   public:
    GraphCut(cv::Mat img);
    void run();

    cv::Mat get_segmentation(SegType type) const;

   private:
    void compute_max_flow();
    void segmention_bfs();
    ScribbleInteractionTool interaction_tool_;
    ImageGraph graph_;
    cv::Mat img_;
};

class AugmentingPath {
   public:
    AugmentingPath(int target_id);
    bool empty();
    bool find_sink();
    std::pair<Node*, Edge*> pop();
    void push(std::pair<Node*, Edge*> edge);
    void update_residual();

   private:
    std::stack<std::pair<Node*, Edge*>> path_;
    double min_residual_;
    bool find_sink_;
    int target_id_;
};
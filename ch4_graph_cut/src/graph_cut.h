#include "image_graph.h"
#include "interaction_tool.h"
#include <stack>

enum class SegType { FOREGROUND = 1, BACKGROUND = 2 };
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
    cv::Mat mask_foreground_;
};

class AugmentingPath {
   public:
    AugmentingPath(int target_id);
    bool empty();
    std::pair<Node*, Edge*> pop();
    void push(const std::pair<Node*, Edge*>& edge);
    void update_residual();

   private:
    std::stack<std::pair<Node*, Edge*>>
        path_;  // todo1 : do not need to save whole path, save target only
    double min_residual_;
    int target_id_;
};

/*--------------------------------------------------------
#####################implementation: AugmentingPath #####################
---------------------------------------------------------*/

// todo1 :1 inline, 2 remove pair 3, change min
inline void AugmentingPath::push(const std::pair<Node*, Edge*>& edge) {
    path_.push(edge);
    // todo compare this min stack with the original min calculation using loop
    min_residual_ = std::min(edge.second->get_residual(), min_residual_);
}
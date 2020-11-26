#include "image_graph.h"
#include "interaction_tool.h"
enum class SegType { FOREGROUND = 1, BACKGOUND = 2 };
class GraphCut {
   public:
    GraphCut(cv::Mat img);
    void run();

    cv::Mat get_segmentation(SegType type) const;

   private:
    ScribbleInteractionTool interaction_tool_;
    ImageGraph graph_;
    cv::Mat img_;
};
#pragma once
#include "graph.h"
#include <opencv2/core.hpp>

class Edge : public EdgeBase {
   public:
    Edge(double weight);

   private:
    double cap_;
    double flow_;
};

typedef NodeBase<Edge> Node;
class ImageGraph : public Graph<Node, Edge> {
   public:
    ImageGraph(cv::Mat img);

   private:
    cv::Mat img_;

    struct Dist {
        Dist() = default;
        double get_weight(int r, int c, int flag) {
            return 0.0f;
        }

        double compute_weight(cv ::Vec3f, cv::Vec3f) {
            return 0.0f;
        }
    } dist;
};

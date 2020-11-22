#pragma once
#include "graph.h"
#include <opencv2/core.hpp>
// class Node {};

// class Edge {};

class ImageGraph : public Graph<Node, Edge> {
   public:
    ImageGraph(cv::Mat img);
    Node* get_root();

   private:
    cv::Mat img_;
};

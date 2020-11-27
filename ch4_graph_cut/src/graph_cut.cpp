#include "graph_cut.h"
#include <queue>
#include <unordered_set>

AugmentingPath BFS_get_path(Node* root, int id_target);

GraphCut::GraphCut(cv::Mat img)
    : interaction_tool_(img),
      graph_(img, interaction_tool_.get_points_foreground(),
             interaction_tool_.get_points_background()),
      img_(img),
      mask_foreground_(cv::Mat::zeros(img.size(), img.type())) {
}
/*--------------------------------------------------------
#####################implementation: Graph Cut #####################
---------------------------------------------------------*/
void GraphCut::run() {
    std::cout
        << " computing max flow through the graph , please waiting ......\n";
    compute_max_flow();
    std::cout << " running segementation ......\n";
    segmention_bfs();
}
cv::Mat GraphCut::get_segmentation(SegType type) const {
    cv::Mat result;
    switch (type) {
        case SegType::FOREGROUND: {
            return mask_foreground_ & img_;
            break;
        }
        case SegType::BACKGROUND: {
            cv::Mat mask_background;
            cv::bitwise_not(mask_foreground_, mask_background);
            return mask_background & img_;
            break;
        }

        default:
            break;
    }
    return result;
}

void GraphCut::compute_max_flow() {
    while (true) {
        // step 1 : do bfs
        // std::stack<std::pair<Node*, Edge*>>
        AugmentingPath path = BFS_get_path(graph_.get_root(), graph_.sink_id_);
        // step 2 :check terminnation
        if (path.empty()) {  // todo remove the find sink
            break;
        }
        // step 3 :update flow
        path.update_residual();
    }
}

void GraphCut::segmention_bfs() {
    std::unordered_set<Node*> visited;

    std::queue<Node*> Q;

    visited.insert(graph_.get_root());
    Q.push(graph_.get_root());

    while (!Q.empty()) {
        Node* curr = Q.front();
        Q.pop();
        auto pos = id_to_pos(curr->id_ - 1, img_.cols);
        mask_foreground_.at<cv::Vec3b>(pos.first, pos.second) = {255, 255, 255};

        for (auto& elem : curr->neighbours_) {
            if (visited.find(elem.first) == visited.end() &&
                std::abs(elem.second.get_residual()) > 1e-20) {
                visited.insert(elem.first);
                Q.push(elem.first);
            }
        }
    }
}
/*--------------------------------------------------------
#####################implementation: Argument Path #####################
---------------------------------------------------------*/
AugmentingPath::AugmentingPath(int target_id)
    : min_residual_(std::numeric_limits<double>::max()),
      target_id_(target_id),
      path_(std::stack<std::pair<Node*, Edge*>>()) {
}

bool AugmentingPath::empty() {
    return path_.empty();
}

std::pair<Node*, Edge*> AugmentingPath::pop() {
    auto edge = path_.top();
    path_.pop();
    return edge;
}

void AugmentingPath::push(std::pair<Node*, Edge*> edge) {
    path_.push(edge);
    // todo compare this min stack with the original min calculation using loop
    min_residual_ = std::min(edge.second->get_residual(), min_residual_);
}

void AugmentingPath::update_residual() {
    while (!path_.empty()) {
        this->pop().second->flow_ += min_residual_;
    }
}

/*--------------------------------------------------------
#####################implementation:BFS get path #####################
---------------------------------------------------------*/

AugmentingPath BFS_get_path(Node* root, int id_target) {
    std::unordered_set<Node*> visited;
    AugmentingPath path(id_target);

    std::queue<Node*> Q;

    visited.insert(root);
    Q.push(root);
    // std::cout << "--------------- one sweep--------------- " << '\n';
    while (!Q.empty()) {
        Node* curr = Q.front();

        if (curr->id_ == id_target) {
            while (curr->id_ != root->id_) {
                path.push(std::pair<Node*, Edge*>(curr, curr->prev_.second));
                std::cout << "edge between : ( " << curr->id_ << " ) and ( "
                          << curr->prev_.first->id_ << " ) ---> "
                          << "flow :" << curr->prev_.second->flow_
                          << ", cap :" << curr->prev_.second->cap_
                          << ", residual :"
                          << curr->prev_.second->get_residual() << '\n';
                curr = curr->prev_.first;
            }
            std::cout << '\n';
        }

        Q.pop();

        for (auto& elem : curr->neighbours_) {
            if (visited.find(elem.first) == visited.end() &&
                !elem.second.is_full()) {
                visited.insert(elem.first);
                Q.push(elem.first);
                elem.first->back_up_prev(curr, &elem.second);
            }
        }
    }
    return path;
}
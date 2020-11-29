#include "graph_cut.h"
#include "tictoc.h"
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
    // preprocessing();
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

void GraphCut::preprocessing() {
    auto& root_neigh = graph_.get_root()->neighbours_;

    for (auto it = root_neigh.begin(); it != root_neigh.end();) {
        auto& edge_from_src = it->second;
        auto& edge_to_sink = it->first->neighbours_.front().second;
        double cap_edge_from_src = edge_from_src->cap_;
        double cap_edge_to_sink = edge_to_sink->cap_;

        if (cap_edge_from_src > cap_edge_to_sink) {
            it->first->neighbours_.erase(it->first->neighbours_.begin());
            edge_from_src->flow_ = cap_edge_to_sink;
            it++;
        } else {
            it = root_neigh.erase(it);
            edge_to_sink->flow_ = cap_edge_from_src;
        }
    }
}
void GraphCut::compute_max_flow() {
    while (true) {
        // step 1 : do bfs
        // std::stack<std::pair<Node*, Edge*>>
        tictoc::tic();
        AugmentingPath path = BFS_get_path(graph_.get_root(), graph_.sink_id_);
        std::cout << " BFS_get_path : " << tictoc::toc() / 1e3 << " ms\n";
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
                std::abs(elem.second->get_residual()) > 1e-20) {
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

void AugmentingPath::update_residual() {
    while (!path_.empty()) {
        this->pop().second->flow_ += min_residual_;
    }

    min_neigh_list_->erase(min_iter_);
}

/*--------------------------------------------------------
#####################implementation:BFS get path #####################
---------------------------------------------------------*/

AugmentingPath BFS_get_path(Node* root, int id_target) {
    std::unordered_set<Node*> visited;  // todo1 : change to cv::Mat
    AugmentingPath path(id_target);

    std::queue<Node*> Q;

    visited.insert(root);
    Q.push(root);
    // std::cout << "--------------- one sweep--------------- " << '\n';
    while (!Q.empty()) {
        Node* curr = Q.front();
        std::cout << " curr id :" << curr->id_ << '\n';
        if (curr->id_ == id_target) {
            while (curr->id_ != root->id_) {
                // path.push(std::pair<Node*, Edge*>(curr, curr->prev_.second));
                path.push(&(curr->prev_.first->neighbours_), );
                std::cout << "edge between : ( " << curr->id_ << " ) and ( "
                          << curr->prev_.first->id_ << " ) ---> "
                          << "flow :" << curr->prev_.second->flow_
                          << ", cap :" << curr->prev_.second->cap_
                          << ", residual :"
                          << curr->prev_.second->get_residual() << '\n';
                curr = curr->prev_.first;
            }
            std::cout << '\n';
            break;
        }

        Q.pop();

        for (auto it = curr->neighbours_.begin(); it != curr->neighbours_.end();
             it++) {
            if ((!it->second->is_full()) &&
                visited.find(it->first) == visited.end()) {
                // add curr elem to Q
                visited.insert(it->first);
                Q.push(it->first);
                it->first->back_up_prev(curr, it->second);

                // check if target is accessable
                auto& edge_to_t = it->first->neighbours_.front();
                if (!edge_to_t.second->is_full() &&
                    visited.find(edge_to_t.first) == visited.end()) {
                    visited.insert(edge_to_t.first);
                    Q.push(edge_to_t.first);
                    edge_to_t.first->back_up_prev(it->first, edge_to_t.second);
                    break;
                }
            }
        }
    }
    return path;
}
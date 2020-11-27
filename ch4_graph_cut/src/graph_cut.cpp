#include "graph_cut.h"
#include <queue>
#include <unordered_set>

AugmentingPath BFS_get_path(Node* root, int id_target);

GraphCut::GraphCut(cv::Mat img)
    : interaction_tool_(img),
      graph_(img, interaction_tool_.get_points_foreground(),
             interaction_tool_.get_points_background()),
      img_(img) {
}
/*--------------------------------------------------------
#####################implementation: Graph Cut #####################
---------------------------------------------------------*/
void GraphCut::run() {
    compute_max_flow();
    // segmention_bfs();
}
cv::Mat GraphCut::get_segmentation(SegType type) const {
    cv::Mat result;
    switch (type) {
        case SegType::FOREGROUND:
            /* code */
            break;
        case SegType::BACKGOUND:
            /* code */
            break;

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
        if (path.empty() || !path.find_sink()) {  // todo remove the find sink
            break;
        }
        // step 3 :update flow
        path.update_residual();
    }
}

/*--------------------------------------------------------
#####################implementation: Argument Path #####################
---------------------------------------------------------*/
AugmentingPath::AugmentingPath(int target_id)
    : min_residual_(std::numeric_limits<double>::max()),
      target_id_(target_id),
      path_(std::stack<std::pair<Node*, Edge*>>()),
      find_sink_(false) {
}

bool AugmentingPath::empty() {
    return path_.empty();
}

bool AugmentingPath::find_sink() {
    return find_sink_;
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
    find_sink_ = (edge.first->id_ == target_id_);
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
    std::cout << "--------------- one sweep--------------- " << '\n';
    while (!Q.empty()) {
        Node* curr = Q.front();

        if (curr->id_ == id_target) {
            while (curr->id_ != root->id_) {
                path.push(std::pair<Node*, Edge*>(curr, curr->prev_.second));
                std::cout << "curr id :" << curr->id_
                          << "flow :" << curr->prev_.second->flow_ << '\n';
                curr = curr->prev_.first;
            }
            /*             std::cout << "curr id :" << curr->id_
                                  << "flow :" << curr->parent_.second->flow_ <<
               '\n' */
        }

        Q.pop();

        for (auto& elem : curr->neighbours_) {
            if (visited.find(elem.first) == visited.end() &&
                !elem.second.is_full()) {
                visited.insert(elem.first);
                Q.push(elem.first);
                elem.first->prev_ = std::pair<Node*, Edge*>(curr, &elem.second);
            }
        }
    }
    return path;
}
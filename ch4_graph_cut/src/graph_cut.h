/**
______________________________________________________________________
*********************************************************************
* @brief This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "image_graph.h"
#include "interaction_tool.h"
#include <stack>

enum class SegType { FOREGROUND = 1, BACKGROUND = 2 };

/**
 * @brief Graph Cut class
 *
 */
class GraphCut {
   public:
    /**
     * @brief Construct a new Graph Cut object
     *
     * @param img
     */
    GraphCut(cv::Mat img);
    /**
     * @brief Exeutable function of graph cut
     *
     */
    void run();
    /**
     * @brief Get the segmentation object
     *
     * @param type
     * @return cv::Mat
     */
    cv::Mat get_segmentation(SegType type) const;

   private:
    /**
     * @brief Apply max flow algorithm to solve the graph cut problem
     *
     */
    void compute_max_flow();
    /**
     * @brief Get segmentation result using bfs
     *
     */
    void segmention_bfs();
    /**
     * @brief calculate the residual for the first round
     *
     */
    void preprocessing();

    ScribbleInteractionTool interaction_tool_;
    ImageGraph graph_;

    cv::Mat img_;
    cv::Mat mask_foreground_;
};

/**
 * @brief AugmentingPath class
 *
 */
class AugmentingPath {
   public:
    /**
     * @brief Construct a new Augmenting Path object given sink id
     *
     * @param target_id
     */
    AugmentingPath(int target_id);
    /**
     * @brief Check if the path is empty
     *
     * @return true
     * @return false
     */
    bool empty();
    /**
     * @brief pop the first element and get it
     *
     * @return std::pair<Node*, Edge*>
     */
    std::pair<Node*, Edge*> pop();
    /**
     * @brief Push the neighbours with minimial residual and its iterator
     *
     * @param min_neigh_list
     * @param min_iter
     */
    void push(std::list<std::pair<Node*, Edge*>>* min_neigh_list,
              std::list<std::pair<Node*, Edge*>>::iterator min_iter);
    /**
     * @brief add min residual with all cap of edge in the path
     *
     */
    void update_residual();

   private:
    std::stack<std::pair<Node*, Edge*>>
        path_;  // todo1 : do not need to save whole path, save target only
    double min_residual_;
    std::list<std::pair<Node*, Edge*>>* min_neigh_list_;
    std::list<std::pair<Node*, Edge*>>::iterator min_iter_;

    int target_id_;
};

/*--------------------------------------------------------
#####################implementation: AugmentingPath #####################
---------------------------------------------------------*/

// todo1 :1 inline, 2 remove pair 3, change min
inline void AugmentingPath::push(
    std::list<std::pair<Node*, Edge*>>* min_neigh_list,
    std::list<std::pair<Node*, Edge*>>::iterator min_iter) {
    path_.push(*min_iter);
    // todo compare this min stack with the original min calculation using loop
    double residual = min_iter->second->get_residual();
    if (residual < min_residual_) {
        min_residual_ = residual;
        min_neigh_list_ = min_neigh_list;
        min_iter_ = min_iter;
    }
}
#include "mean_shift_tracker.h"
#include "motion_predictor.h"
#include "opencv_utils.h"
#include <iostream>

MeanShiftTracker::MeanShiftTracker() {
    TrackerDataBase* db_raw_ptr = new TrackerDataBase();
    db_ptr_ = std::shared_ptr<TrackerDataBase>(db_raw_ptr);
    ms_ = MeanShift(std::shared_ptr<DataBase>(db_raw_ptr));
}

void MeanShiftTracker::process(const std::vector<cv::Mat>& video, const cv::Mat temp) {
    cv::Point2i up_left = template_matching(video[0], temp);

    cv::Mat vis = video[0].clone();
    draw_bounding_box_vis_image(vis, up_left.x, up_left.y, temp.cols, temp.rows);
    //! change the type of ul to 2f
    up_left = cv::Point2f(up_left.x, up_left.y);
    db_ptr_->set_template(temp);

    MotionPredictor motion_predictor(static_cast<cv::Point2f>(up_left));
    std::cout << motion_predictor.get_curr_pos() << '\n';
    for (int i = 0; i < video.size(); i++) {
        std::cout << "Set the " << i << "-th img" << '\n';
        db_ptr_->set_img(video[i]);
        cv::Point2f next_pos = motion_predictor.next_pos();
        db_ptr_->set_obj_predicted_initial_center(next_pos);
        std::cout << "|| The predicted motion is: " << next_pos.x << " " << next_pos.y << '\n';

        ms_.run(100);

        cv::Point2f new_pos = db_ptr_->get_object_center();

        motion_predictor.set_tracking_result(new_pos);
        /* code */
    }
}
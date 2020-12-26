#include "mean_shift_tracker.h"
#include "motion_predictor.h"
#include "opencv_utils.h"

MeanShiftTracker::MeanShiftTracker() {
    TrackerDataBase* db_raw_ptr = new TrackerDataBase(cv::Mat(), cv::Mat(), cv::Point2f());
    db_ptr_ = std::shared_ptr<TrackerDataBase>(db_raw_ptr);
    ms_ = MeanShift(std::shared_ptr<DataBase>(db_raw_ptr));
}

void MeanShiftTracker::process(const std::vector<cv::Mat>& video, const cv::Mat temp) {
    cv::Point2i up_left = template_matching(video[0], temp);
    cv::Mat vis = video[0].clone();
    draw_bounding_box_vis_image(vis, up_left.x, up_left.y, temp.cols, temp.rows);

    db_ptr_->set_template(temp);

    MotionPredictor motion_predictor(static_cast<cv::Point2f>(up_left));
    for (int i = 0; i < video.size(); i++) {
        db_ptr_->set_img(video[i]);
        db_ptr_->set_pos(motion_predictor.next_pos());

        ms_.run(100);

        cv::Point2f new_pos = db_ptr_->get_tracking_result();

        motion_predictor.set_tracking_result(new_pos);
        /* code */
    }
}
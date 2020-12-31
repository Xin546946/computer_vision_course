/**
______________________________________________________________________
*********************************************************************
* @brief  This file is developed for the course of ShenLan XueYuan:
* Fundamental implementations of Computer Vision
* all rights preserved
* @author Xin Jin, Zhaoran Wu
* @contact: xinjin1109@gmail.com, zhaoran.wu1@gmail.com
*
______________________________________________________________________
*********************************************************************
**/
#include "bounding_box.h"
#include "motion_predictor.h"
#include "ms_tracking.h"

class MeanShiftTracker {
   public:
    /**
     * @brief Construct a new Mean Shift Tracker object
     *
     * @param [in] temp template for mean shift tracking
     * @param [in] init_bbox init position of the object
     */
    MeanShiftTracker(cv::Mat temp, BoundingBox init_bbox) : motion_predictor(init_bbox.center()), ms_tracking(temp) {
    }

    /**
     * @brief tracking the object inside the video
     *
     * @param [in] video
     */
    void process(const std::vector<cv::Mat>& video);

   private:
    MotionPredictor motion_predictor;  // predict the init position according to constant motion model
    MeanShiftTracking ms_tracking;     // tracking the obeject with mean shift
};
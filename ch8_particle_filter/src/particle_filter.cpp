#include "particle_filter.h"
#include "math_utils.h"

ParticleFilter::ParticleFilter(cv::Mat temp, const BoundingBox& init_bbox, int num_particles, int num_histogramm_bins)
    : motion_model_(init_bbox.center()), hist_temp_(num_histogramm_bins, 0.0, 255.0) {
    cv::Mat temp_64f;
    temp.convertTo(temp_64f, CV_64FC1);
    hist_temp_ = make_histogramm(temp_64f, num_histogramm_bins, cv::Mat::ones(temp.size(), CV_64FC1), 0.0, 255.0);

    init_particles(init_bbox, num_particles);
}

void ParticleFilter::init_particles(const BoundingBox& init_bbox, int num_particles) {
    cv::Point2f center = init_bbox.center();
    std::array<double, 4> means{init_bbox.width(), init_bbox.height(), center.x, center.y};
    std::array<double, 4> stddev{5.0, 5.0, 15.0, 15.0};

    auto gauss_datas = generate_gauss_data<double, 4>(num_particles, means, stddev);

    for (const auto& data : gauss_datas) {
        // each data contains {w,h,x,y}
        particles_.emplace_back(data[0], data[1], data[2], data[3], 1.0);
    }
}

void ParticleFilter::update_status() {
}

void ParticleFilter::update_weights(cv::Mat frame) {
}

void ParticleFilter::resampling() {
}

Particle::Particle(float w, float h, float x_center, float y_center, float weight)
    : state_(w, h, x_center, y_center), weight_(weight) {
}

State::State(float w, float h, float x_center, float y_center) : bbox_(w, h, x_center, y_center) {
}

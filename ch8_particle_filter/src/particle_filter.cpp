#include "particle_filter.h"
#include "math_utils.h"
#include "opencv_utils.h"

double compute_weight(const Histogram& temp_hist, const Histogram& candidata_hist, double sigma_square_inv = 2.5e-5);

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
    cv::Vec2f delta_motion = motion_model_.predict_motion();
    std::array<double, 4> means{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> stddev{3.0, 3.0, 2.0, 2.0};
    auto noises = generate_gauss_data<double, 4>(particles_.size(), means, stddev);

    for (int i = 0; i < particles_.size(); i++) {
        particles_[i].update_with_motion_and_noise(delta_motion, noises[i]);
    }
}

void ParticleFilter::update_weights(cv::Mat frame) {
    cv::Mat frame_64f;
    frame.convertTo(frame_64f, CV_64FC1);

    for (auto& particle : particles_) {
        const State& state = particle.state_;
        cv::Mat sub_img = get_sub_image_around(frame_64f, state.x_center(), state.y_center(), state.w(), state.h());
        Histogram hist_sub_img =
            make_histogramm(sub_img, hist_temp_.num_bin(), cv::Mat::ones(sub_img.size(), CV_64FC1), 0.0, 255.0);

        particle.weight_ = compute_weight(hist_temp_, hist_sub_img, 2e-5);
    }
}

void ParticleFilter::resampling() {
}

Particle::Particle(float w, float h, float x_center, float y_center, float weight)
    : state_(w, h, x_center, y_center), weight_(weight) {
}

State::State(float w, float h, float x_center, float y_center) : bbox_(w, h, x_center, y_center) {
}

void Particle::update_with_motion_and_noise(cv::Vec2f delta_motion, std::array<double, 4> noise) {
    cv::Size2f bbox_size = state_.size();

    state_.resize(bbox_size.width + noise[0], bbox_size.height + noise[1]);

    state_.move(delta_motion(0) + noise[2], delta_motion(1) + noise[3]);
}

double compute_weight(const Histogram& temp_hist, const Histogram& candidata_hist, double sigma_square_inv) {
    assert(candidata_hist.num_bin() == temp_hist.num_bin());
    double dist_saqure = 0.0;

    for (int i = 0; i < temp_hist.num_bin(); i++) {
        dist_saqure += std::pow(temp_hist.get_bin_height(i), candidata_hist.get_bin_height(i));
    }

    double w = std::exp(-0.5 * sigma_square_inv * dist_saqure);
    return w;
}
State ParticleFilter::compute_mean_state() {
    float w = 0.0;
    float h = 0.0;
    float x = 0.0;
    float y = 0.0;

    for (const auto& particle : particles_) {
        w += particle.state_.w();
        h += particle.state_.h();
        x += particle.state_.x_center();
        y += particle.state_.y_center();
    }

    int size = particles_.size();
    w /= size;
    h /= size;
    x /= size;
    y /= size;

    return State(w, h, x, y);
}

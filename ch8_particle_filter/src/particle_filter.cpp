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
#include "particle_filter.h"
#include "math_utils.h"
#include "opencv_utils.h"

double compute_weight_factor(const Histogram& temp_hist, const Histogram& candidata_hist,
                             double sigma_square_inv = 2.5e-5);

ParticleFilter::ParticleFilter(cv::Mat temp, const BoundingBox& init_bbox, int num_particles, int num_histogramm_bins)
    : motion_model_(init_bbox.center()), hist_temp_(num_histogramm_bins, 0.0, 255.0) {
    cv::Mat temp_64f;
    temp.convertTo(temp_64f, CV_64FC1);

    hist_temp_ = make_histogramm(temp_64f, num_histogramm_bins, cv::Mat::ones(temp.size(), CV_64FC1), 0.0, 255.0);
    hist_temp_.equalize();
    init_particles(init_bbox, num_particles);
}

void ParticleFilter::init_particles(const BoundingBox& init_bbox, int num_particles) {
    // todo initialize the particles
    // todo you can use generate_gauss_data to generate random numbers as noise
}

void ParticleFilter::predict_status() {
    // todo predict the status
    // todo you can use generate_gauss_data and update_with_motion_and_noise
}

void ParticleFilter::update_weights(cv::Mat frame) {
    cv::Mat frame_64f;
    frame.convertTo(frame_64f, CV_64FC1);  // CV_64FC1 form for calculation
    // todo update weight
    // todo use get_sub_image_around, make_histogramm, compute_weight_factor
}

void ParticleFilter::resampling() {
    // todo resample the particles
}

Particle::Particle(float w, float h, float x_center, float y_center, float weight)
    : state_(w, h, x_center, y_center), weight_(weight) {
}

State::State(float w, float h, float x_center, float y_center) : bbox_(x_center - w / 2, y_center - h / 2, w, h) {
}

void Particle::update_with_motion_and_noise(cv::Vec2f delta_motion, const std::array<double, 4>& noise) {
    cv::Size2f bbox_size = state_.size();

    state_.resize(bbox_size.width + noise[0], bbox_size.height + noise[1]);

    state_.move(delta_motion(0) + noise[2], delta_motion(1) + noise[3]);
}

double compute_weight_factor(const Histogram& temp_hist, const Histogram& candidata_hist, double sigma_square_inv) {
    assert(candidata_hist.num_bin() == temp_hist.num_bin());
    double chi_saqure_dist = 0.0;
    for (int i = 0; i < temp_hist.num_bin(); i++) {
        double x = temp_hist.get_bin_height(i);
        double y = candidata_hist.get_bin_height(i);
        chi_saqure_dist += std::pow(x - y, 2.0) / (x + y + 1e-20);
    }

    double factor = std::exp(-0.5 * sigma_square_inv * chi_saqure_dist);
    return factor;
}
State ParticleFilter::compute_mean_state_and_set_observation() {
    float w = 0.0f;
    float h = 0.0f;
    float x = 0.0f;
    float y = 0.0f;

    double sum_w = 0.0;

    for (const auto& particle : particles_) {
        if (particle.bad_) {
            continue;
        }

        const State& state = particle.state_;

        w += particle.weight_ * state.w();
        h += particle.weight_ * state.h();
        x += particle.weight_ * state.x_center();
        y += particle.weight_ * state.y_center();
        sum_w += particle.weight_;
    }

    w /= sum_w;
    h /= sum_w;
    x /= sum_w;
    y /= sum_w;

    State mean_state(w, h, x, y);

    motion_model_.set_observation(mean_state.center());

    return mean_state;
}

Particle::Particle(State state, double weight) : state_(state), weight_(weight) {
}

void ParticleFilter::visualize(cv::Mat frame) {
    cv::Mat vis;
    cv::cvtColor(frame, vis, CV_GRAY2BGR);

    for (auto p : particles_) {
        cv::circle(vis, p.state_.center(), 1, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("particles:", vis);
    cv::waitKey(1);
}

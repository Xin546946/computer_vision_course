#include "particle_filter.h"
#include "math_utils.h"
#include "opencv_utils.h"
#include "tictoc.h"
#include <future>
#include <thread>

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
    cv::Point2f center = init_bbox.center();
    std::array<double, 4> means{init_bbox.width(), init_bbox.height(), center.x, center.y};
    std::array<double, 4> stddev{2.0, 2.0, 10.0, 10.0};

    auto gauss_datas = generate_gauss_data<double, 4>(num_particles, means, stddev);
    particles_.reserve(num_particles);
    for (const auto& data : gauss_datas) {
        // each data contains {w,h,x,y}
        particles_.emplace_back(data[0], data[1], data[2], data[3], 1.0);
    }
}

void ParticleFilter::predict_status() {
    cv::Vec2f delta_motion = motion_model_.predict_motion();

    // w,h,x,y
    std::array<double, 4> means{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> stddev{0.1, 0.1, 1.0, 1.0};
    auto noises = generate_gauss_data<double, 4>(particles_.size(), means, stddev);
#pragma omp parallel for
    for (int i = 0; i < particles_.size(); i++) {
        particles_[i].update_with_motion_and_noise(delta_motion, noises[i]);
    }
}

void ParticleFilter::update_weights(cv::Mat frame) {
    cv::Mat frame_64f;
    frame.convertTo(frame_64f, CV_64FC1);

#pragma omp parallel for
    for (int i = 0; i < particles_.size(); i++) {
        const State& state = particles_[i].state_;
        cv::Mat sub_frame_64f =
            get_sub_image_around(frame_64f, state.x_center(), state.y_center(), state.w(), state.h());
        cv::Mat sub_img_vis = get_sub_image_around(frame, state.x_center(), state.y_center(), state.w(), state.h());

        if (sub_frame_64f.cols == 0 || sub_frame_64f.rows == 0) {
            particles_[i].bad_ = true;
            continue;
        }

        // cv::imshow("sub", sub_img_vis);
        // cv::waitKey(0);

        Histogram hist_sub_frame = make_histogramm(sub_frame_64f, hist_temp_.num_bin(),
                                                   cv::Mat::ones(sub_frame_64f.size(), CV_64FC1), 0.0, 255.0);
        hist_sub_frame.equalize();

        particles_[i].weight_ *= compute_weight_factor(hist_temp_, hist_sub_frame, 20);
        std::cerr << "weight : " << particles_[i].weight_ << '\n';
    }
}

void ParticleFilter::resampling() {
    tictoc::tic();
    // todo : using std mutilple thread,  1 +　．．．　＋１００；
    double integration = 0.0;
    std::vector<std::pair<double, int>> integration_to_id;
    integration_to_id.reserve(particles_.size());

    for (int i = 0; i < particles_.size(); i++) {
        if (particles_[i].bad_) {
            continue;
        }

        integration += particles_[i].weight_;
        integration_to_id.emplace_back(integration, i);
    }

    std::vector<Particle> new_particles;
    new_particles.reserve(particles_.size());

    for (int i = 0; i < particles_.size(); i++) {
        float rnd_num = generate_random_data(0.0f, integration);
        auto it_candidate = std::lower_bound(integration_to_id.begin(), integration_to_id.end(), rnd_num,
                                             [](std::pair<double, int> lhs, double rhs) { return lhs.first < rhs; });
        int index_candidate = it_candidate->second;
        new_particles.emplace_back(particles_[index_candidate].state_, 1.0f);
    }

    particles_ = new_particles;
    std::cout << "Resampling costs " << tictoc::toc() / 1e6 << "miliseconds" << '\n';
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
// State ParticleFilter::compute_mean_state_and_set_observation() {
Particle ParticleFilter::compute_state_from_single_thread(int from, int to) {
    float w = 0.0f;
    float h = 0.0f;
    float x = 0.0f;
    float y = 0.0f;

    double sum_w = 0.0;

    // todo : using std mutilple thread,  1 + ... + 100　；
    for (int i = from; i < to; i++) {
        if (particles_[i].bad_) {
            continue;
        }

        const State& state = particles_[i].state_;

        w += particles_[i].weight_ * state.w();
        h += particles_[i].weight_ * state.h();
        x += particles_[i].weight_ * state.x_center();
        y += particles_[i].weight_ * state.y_center();
        sum_w += particles_[i].weight_;
    }
    w /= sum_w;
    h /= sum_w;
    x /= sum_w;
    y /= sum_w;
    return Particle(w, h, x, y, sum_w / (to - from));
}

State ParticleFilter::multithread_compute_mean_state_and_set_observation() {
    std::vector<std::future<Particle>> result;
    int num_thread = static_cast<int>(std::thread::hardware_concurrency());
    for (int i = 0; i < num_thread - 1; i++) {
        std::future<Particle> sum_single_thread =
            std::async(std::launch::async, &ParticleFilter::compute_state_from_single_thread, this,
                       i * int(particles_.size()) / num_thread, (i + 1) * int(particles_.size()) / num_thread);
        result.push_back(std::move(sum_single_thread));
    }

    double sum = 0.0;
    float w = 0.0f;
    float h = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    for (int i = 0; i < num_thread; i++) {
        w += result[i].get().state_.w();
        h += result[i].get().state_.h();
        x += result[i].get().state_.x_center();
        y += result[i].get().state_.y_center();
        sum += result[i].get().weight_;
    }

    w /= sum;
    h /= sum;
    x /= sum;
    y /= sum;

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
    cv::waitKey(0);
}

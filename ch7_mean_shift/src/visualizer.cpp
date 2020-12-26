#include "visualizer.h"
#include "data_base.h"
#include "opencv_utils.h"
#include <thread>

void Visualizer::init() {
    const float w = 2560;
    const float h = 1920;

    pangolin::CreateWindowAndBind("MeanShift Visualizer", w, h);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    cam_3d_ =
        pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(w, h, 1000.0, 1000.0, w / 2.0, h / 2.0, 0.0001, 10.0),
                                    pangolin::ModelViewLookAt(2.0, 1.0, 1.8, 0.1, 0.1, 0.1, pangolin::AxisZ));

    // add 3D view(main)
    const int w_ui = 100;
    view_3d_ = pangolin::CreateDisplay()
                   .SetBounds(0.0, 1.0, pangolin::Attach::Pix(w_ui), 1.0, -w / h)
                   .SetHandler(new pangolin::Handler3D(cam_3d_));

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(w_ui));
}

void Visualizer::show(int rows, int cols) {
    pangolin::Var<bool> menuStop("menu.Stop", true, true);
    init();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        view_3d_.Activate(cam_3d_);

        draw_points();
        draw_frame();
        show_segmentation(rows, cols);

        pangolin::FinishFrame();

        if (menuStop != stop_) {
            std::lock_guard<std::mutex> lg_stop(stop_mutex_);
            if (menuStop != stop_) {
                stop_ = menuStop;
            }
        }
    }
}

void Visualizer::draw_points() {
    glPointSize(6);
    glBegin(GL_POINTS);

    points_mutex_.lock();
    std::vector<cv::Vec3f> points = points_;
    points_mutex_.unlock();

    for (const cv::Vec3d& p : points) {
        glColor3f(p[0], p[1], p[2]);
        glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
}
void Visualizer::draw_frame() const {
    std::vector<line> lines;

    cv::Vec3d p1(0.0, 0.0, 0.0), p2(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0), p4(0.0, 0.0, 1.0);
    cv::Vec3d p5(1.0, 0.0, 1.0), p6(0.0, 1.0, 1.0), p7(1.0, 1.0, 0.0), p8(1.0, 1.0, 1.0);

    lines.push_back({p1, p2});
    lines.push_back({p4, p5});
    lines.push_back({p6, p8});
    lines.push_back({p3, p7});

    lines.push_back({p1, p4});
    lines.push_back({p3, p6});
    lines.push_back({p7, p8});
    lines.push_back({p2, p5});

    lines.push_back({p1, p3});
    lines.push_back({p2, p7});
    lines.push_back({p5, p8});
    lines.push_back({p4, p6});

    draw_lines_with_interpolated_color(lines);
}

void Visualizer::set_data(const std::vector<cv::Vec3f>& bgr_datas) {
    points_mutex_.lock();
    points_.clear();
    points_.reserve(bgr_datas.size());
    std::transform(bgr_datas.begin(), bgr_datas.end(), std::back_inserter(points_),
                   [](const cv::Vec3f& p) { return cv::Vec3f(p[2] / 255.0, p[1] / 255.0, p[0] / 255.0); });
    pos_.clear();
    points_mutex_.unlock();
}

void Visualizer::set_data(const std::vector<BGR>& bgr_datas, const std::vector<cv::Point>& pos) {
    points_mutex_.lock();
    points_.clear();
    points_.reserve(bgr_datas.size());
    std::transform(bgr_datas.begin(), bgr_datas.end(), std::back_inserter(points_),
                   [](const BGR& p) { return cv::Vec3f(p[2] / 255.0, p[1] / 255.0, p[0] / 255.0); });
    pos_ = pos;
    points_mutex_.unlock();
}

void draw_lines_with_interpolated_color(const std::vector<line>& lines) {
    glLineWidth(5);
    glBegin(GL_LINES);
    for (int i = 0; i < lines.size(); i++) {
        glColor3f(lines[i].first[0], lines[i].first[1], lines[i].first[2]);
        glVertex3f(lines[i].first[0], lines[i].first[1], lines[i].first[2]);
        glColor3f(lines[i].second[0], lines[i].second[1], lines[i].second[2]);
        glVertex3f(lines[i].second[0], lines[i].second[1], lines[i].second[2]);
    }
    glEnd();
}

void Visualizer::show_segmentation(int rows, int cols) {
    points_mutex_.lock();
    std::vector<cv::Vec3f> points = points_;
    std::vector<cv::Point> pos = pos_;
    points_mutex_.unlock();

    if (points.empty()) return;

    cv::Mat vis(cv::Size(cols, rows), CV_8UC3);
    if (pos.empty()) {
        for (int r = 0; r < vis.rows; r++) {
            for (int c = 0; c < vis.cols; c++) {
                int id = pos_to_id(r, c, cols);
                vis.at<cv::Vec3b>(r, c) = points[id] * 255;
            }
        }
    } else {
        for (int i = 0; i < pos.size(); i++) {
            vis.at<cv::Vec3b>(pos[i].y, pos[i].x) = points[i] * 255;
        }
    }

    cv::cvtColor(vis, vis, cv::COLOR_RGB2BGR);
    cv::imshow("segementaion", vis);
    cv::waitKey(1);
}

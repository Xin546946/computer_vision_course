#include "visualizer.h"
#include <thread>

Visualizer::Visualizer() {
    // launch rendering loop with thread
    std::thread vis_thread(&Visualizer::show, std::ref(*this));
    vis_thread.detach();
}

void Visualizer::init() {
    const float w = 2560;
    const float h = 1920;

    pangolin::CreateWindowAndBind("MeanShift Visualizer", w, h);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    cam_3d_ = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(w, h, 600, 600, w / 2.0, h / 2.0, 0.0001, 10.0),
                                          pangolin::ModelViewLookAt(-1, -1, -1, 0, 0, 0, pangolin::AxisNegY));

    // add 3D view(main)
    const int w_ui = 100;
    view_3d_ = pangolin::CreateDisplay()
                   .SetBounds(0.0, 1.0, pangolin::Attach::Pix(w_ui), 1.0, -w / h)
                   .SetHandler(new pangolin::Handler3D(cam_3d_));

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(w_ui));
}

void Visualizer::show() {
    init();
    pangolin::Var<bool> menuStop("menu.Stop", false, false);
    int id_pose = 0;
    int num_writed_pose = 0;
    while (!pangolin::ShouldQuit()) {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (menuStop) {
            break;
        }

        view_3d_.Activate(cam_3d_);

        draw_points();

        pangolin::FinishFrame();
    }
    pangolin::Quit();
}

void Visualizer::draw_points() const {
    glPointSize(4);
    glBegin(GL_POINTS);

    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 1, 1);
    for (const cv::Vec3d& p : points_) {
        glColor3f(p[0], p[1], p[2]);
        glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();
}

void Visualizer::set_data(const std::vector<cv::Vec3d>& bgr_datas) {
    std::lock_guard<std::mutex> points_lg(points_mutex_);
    points_.clear();
    points_.reserve(bgr_datas.size());
    std::transform(bgr_datas.begin(), bgr_datas.end(), std::back_inserter(points_),
                   [](const cv::Vec3d& p) { return cv::Vec3f(p[2] / 255.0, p[1] / 255.0, p[0] / 255.0); });
}

void Visualizer::shut() {
    pangolin::Quit();
}

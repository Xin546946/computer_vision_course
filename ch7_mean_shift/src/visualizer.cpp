#include "visualizer.h"
#include <thread>

void Visualizer::init() {
    const float w = 2560;
    const float h = 1920;

    pangolin::CreateWindowAndBind("MeanShift Visualizer", w, h);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    cam_3d_ = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(w, h, 600, 600, w / 2.0, h / 2.0, 0.0001, 10.0),
                                          pangolin::ModelViewLookAt(2, 1, 2, 0, 0, 0, pangolin::AxisZ));

    // add 3D view(main)
    const int w_ui = 100;
    view_3d_ = pangolin::CreateDisplay()
                   .SetBounds(0.0, 1.0, pangolin::Attach::Pix(w_ui), 1.0, -w / h)
                   .SetHandler(new pangolin::Handler3D(cam_3d_));

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(w_ui));
}

void Visualizer::show() {
    init();
    pangolin::Var<bool> menuExit("menu.Exit", false, false);
    int id_pose = 0;
    int num_writed_pose = 0;
    while (!pangolin::ShouldQuit()) {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (menuExit) {
            break;
        }

        view_3d_.Activate(cam_3d_);

        draw_points();
        draw_frame();
        pangolin::glDrawAxis(5);

        pangolin::FinishFrame();
    }
    shut();
}

void Visualizer::draw_points() const {
    glPointSize(6);
    glBegin(GL_POINTS);

    for (const cv::Vec3d& p : points_) {
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

void Visualizer::set_data(const std::vector<cv::Vec3d>& bgr_datas) {
    std::lock_guard<std::mutex> points_lg(points_mutex_);
    points_.clear();
    points_.reserve(bgr_datas.size());
    std::transform(bgr_datas.begin(), bgr_datas.end(), std::back_inserter(points_),
                   [](const cv::Vec3d& p) { return cv::Vec3f(p[2] / 255.0, p[1] / 255.0, p[0] / 255.0); });
}

void Visualizer::shut() {
    if (pangolin::ShouldQuit()) {
        pangolin::Quit();
    }
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
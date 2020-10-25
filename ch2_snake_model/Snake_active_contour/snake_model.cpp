
#include <algorithm>
#include <chrono>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdio.h>
#include <stdio.h> /* printf */
#include <string>
#include <vector>

using namespace std;
using namespace cv;

vector<Point> create_point_example() {
  vector<Point> points;
  Point p;
  for (size_t i = 1; i < 6; i++) {
    p.x = i;
    p.y = i;
    points.push_back(p);
  }
  return points;
}


Mat external_force_image(Mat img, const float w_line, const float w_edge,
                         const float w_term, const float sigma) {
  Mat Ix, Iy, Ixx, Iyy, Ixy;
  // Gaussian blur
  Mat image;
  GaussianBlur(img, image, Size(3, 3), 1, 1);
  image.convertTo(image, CV_32F);

  // calculate the image first derivative w.r.t. x, y as well as the second
  // derivative w.r.t xx,yy,xy
  Sobel(image, Ix, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
  Sobel(image, Iy, CV_32F, 0, 1, 3);
  Sobel(Ix, Ixx, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
  Sobel(Iy, Iyy, CV_32F, 0, 1, 3);
  Sobel(Ix, Ixy, CV_32F, 0, 1, 3);

  Mat e_line, e_edge, e_term, e_extern;
  GaussianBlur(image, e_line, Size(3, 3), 1, 1);
  // Calculate (Iyy.*Ix.*Ix - 2*Ixy.*Ix.*Iy + Ixx.*Iy.*Iy)./
  // ((1+Ix.*Ix+Iy.*Iy)**1.5) tmp1 = Iyy.*Ix.*Ix tmp2 = 2*Ixy.*Ix.*Iy tmp3 =
  // Ixx.*Iy.*Iy tmp4 = (1+Ix.*Ix+Iy.*Iy)
  Mat tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
  multiply(Ix, Ix, tmp1);
  multiply(Ix, Ix, tmp4);
  multiply(tmp1, Iyy, tmp1);
  multiply(Ix, Iy, tmp2);
  multiply(tmp2, -2 * Ixy, tmp2);
  multiply(Iy, Iy, tmp3);
  multiply(Iy, Iy, tmp5);
  multiply(tmp3, Ixx, tmp3);

  e_term = tmp1 + tmp2 + tmp3;

  // cout << one_matrix << endl;
  Mat tes = tmp5 + tmp4 + 1;
  // cout << tes << endl;

  pow(tmp5 + tmp4 + 1, 1.5, tmp6);

  divide(e_term, tmp6, e_term);

  sqrt((tmp5 + tmp4), e_edge);

  e_extern = w_line * e_line - w_edge * e_edge - w_term * e_term;
  /*namedWindow("e_line", WINDOW_AUTOSIZE);
  imshow("e_line", e_line);
  waitKey(0);
  namedWindow("e_edge", WINDOW_AUTOSIZE);
  imshow("e_edge", e_edge);
  waitKey(0);
  namedWindow("e_term", WINDOW_AUTOSIZE);
  imshow("e_term", e_term);
  waitKey(0);

  namedWindow("e_extern",WINDOW_AUTOSIZE);
  imshow("e_extern",e_extern);
  waitKey(0); */
  return e_extern;
}

void min_max(Mat &x, Mat &y, vector<Point> &C, double num, Mat u) {
  // int size_1, size_2;
  // size_1, size_2 = u.size();
  cout << "The size of u is: " << u.size() << endl;
  size_t size_1 = u.rows;
  size_t size_2 = u.cols;

  // double tmp_1, tmp_2;
  for (int i{0}; i < C.size(); i++) {
    x.at<float>(i) = x.at<float>(i) > num ? x.at<float>(i) : num;
    x.at<float>(i) = x.at<float>(i) < size_1 ? x.at<float>(i) : size_1;
    y.at<float>(i) = y.at<float>(i) > num ? y.at<float>(i) : num;
    y.at<float>(i) = y.at<float>(i) < size_2 ? y.at<float>(i) : size_2;
  }
}

Mat circshift(Mat input, int down_shift) {
  // circularly shifts the values in the array input by downshift elements.
  down_shift = ((down_shift % input.rows) + input.rows) % input.rows;
  Mat output = Mat::zeros(input.rows, input.cols, input.type());

  for (int i = 0; i < input.rows; i++) {
    int new_row = (i + down_shift) % input.rows;
    // cout << "new row" << newrow << endl;
    for (int j = 0; j < input.cols; j++) {
      int new_column = j % input.cols;
      output.at<int>(new_row, j) = input.at<int>(i, j);
    }
  }
  return output;
}

void contour_initialize_1(int center_x, int center_y, int radius,
                          Mat &contour_img,
                          vector<vector<Point>> &snake_coords) {
  Point center(center_x, center_y);
  circle(contour_img, center, radius, 255, 1, 8);

  findContours(contour_img, snake_coords, CV_RETR_EXTERNAL,
               CV_CHAIN_APPROX_NONE);
}

void dispImage(Mat& img, String windowName, int delay) {

  if(img.empty()) { //Read image and display after checking for image validity
    cout << "Error reading image File!";
    cin.ignore();
  }else {
    namedWindow(windowName, 0);
    imshow(windowName, img);
    waitKey(delay);
  }
}

void normalizeMat(Mat &fx, Mat &fy) {
  Mat mag(fx.size(), CV_32F);
  Mat prod_temp1, prod_temp2;

  // Magnitude Computation
  multiply(fx, fx, prod_temp1);
  multiply(fy, fy, prod_temp2);
  mag = prod_temp1 + prod_temp2;
  sqrt(mag, mag);

  fx = fx / mag;
  fy = fy / mag;
}

void drawOptFlowMap(Mat &fx, Mat &fy, Mat &cflowmap, int step,
                    double scaleFactor, Scalar &color) {
  for (int y = 0; y < cflowmap.rows; y += step)
    for (int x = 0; x < cflowmap.cols; x += step) {
      Point2f fxy;
      fxy.y = fy.at<float>(y, x);
      fxy.x = fx.at<float>(y, x);

      if (fxy.x != 0 || fxy.y != 0) {
        line(cflowmap, Point(x, y),
             Point(cvRound(x + (fxy.x) * scaleFactor),
                   cvRound(y + (fxy.y) * scaleFactor)),
             color);
      }
      circle(cflowmap, Point(x, y), 1, color, -1);
    }
}

void grad_vector_field(Mat &u, Mat &v, float mu, int ITER) {
  Mat sqrMag, u2, v2;
  Mat u0, v0;
  u.copyTo(u0);
  v.copyTo(v0);
  Mat u_term2, v_term2;
  Mat gradX, gradY;
  // square magnitude of force field
  multiply(u, u, u2);
  multiply(v, v, v2);
  sqrMag = u2 + v2;

  // cout << sqrMag << endl;
  int dt;
  dt = 1;

  // Solve the differential equation iteratively//
  // Diffusion function being implemented is from the paper Snakes, Shapes and
  // Gradient Vector FLow by Chenyang Xu and Jerry L Prince
  // u = u + mu * del2(u) -(u - fx) * (fx2 + fy2);
  // v = v + mu * del2(v) - (v - fy) * (fx2 + fy2);
  Mat img = imread(
      "/home/kit/CLionProjects/Snake_active_contour/build/testimage.png", 0);
  double sigma = 20;
  for (int i = 0; i < ITER; i++) {
    GaussianBlur(u, u, Size(3, 3), sigma, sigma);
    GaussianBlur(v, v, Size(3, 3), sigma, sigma);
    Laplacian(u, gradX, CV_32F, 1, BORDER_REFLECT);
    Laplacian(v, gradY, CV_32F, 1, BORDER_REFLECT);
    Mat u_term2, v_term2;
    multiply(sqrMag, (u - u0), u_term2);
    multiply(sqrMag, (v - v0), v_term2);
    u = u + dt * (mu * gradX - u_term2);
    v = v + dt * (mu * gradY - v_term2);
    /*****visualization of u and v for each iteration ******/
    /*normalizeMat(u,v);
    Mat dem_img;
    cvtColor(img, dem_img, CV_GRAY2RGB);
    Scalar color(0,255,0);
    cout << 1111 << endl;
    drawOptFlowMap(v, u, dem_img, 8,8, color);
    namedWindow("Grad Field_gvfi", 0);
    imshow("Grad Field_gvfi", dem_img);
    waitKey(0);

    namedWindow("iterated u", 0);
    imshow("iterated u", u);
    waitKey(0);
    namedWindow("iterated v", 0);
    imshow("iterated v", v);
    waitKey(0);*/
  }
  normalizeMat(u, v);
}


void snake_internal_force_matrix2d(double alpha, double beta, double gamma,
                                   Mat &internal_force_matrix, int num_points) {
  Mat b = (Mat_<double>(1, 5) << beta, -(alpha + 4 * beta),
           (2 * alpha + 6 * beta), -(alpha + 4 * beta), beta);
  Mat A = Mat::eye(num_points, num_points, CV_32F);

  internal_force_matrix = *b.ptr<double>(0, 0) * circshift(A, 2);
  internal_force_matrix += *b.ptr<double>(0, 1) * circshift(A, 1);
  internal_force_matrix += *b.ptr<double>(0, 2) * A;

  internal_force_matrix += *b.ptr<double>(0, 3) * circshift(A, -1);
  internal_force_matrix += *b.ptr<double>(0, 4) * circshift(A, -2);
  internal_force_matrix = internal_force_matrix + gamma * A;
  internal_force_matrix = internal_force_matrix.inv(DECOMP_CHOLESKY);
}


vector<Point> get_contour_normal(vector<Point> C) {
  int a = 4; // use the n'th neighbour to calculate the normal
  size_t n = C.size();

  vector<int> index_f, index_b;
  vector<Point> contour_normal;
  for (int i{1}; i < n + 1; i++) {
    if (i + a > n)
      index_f.push_back(i + a - n);
    else
      index_f.push_back(i + a);

    if (i - a < 1)
      index_b.push_back(i - a + n);
    else
      index_b.push_back(i - a);
  }
  Point dx;
  double l = 0.0;
  Point tmp_p;
  for (int i = 0; i < n; i++) {
    dx.x = C.at(index_f.at(i)).x - C.at(index_b.at(i)).x;
    dx.y = C.at(index_f.at(i)).y - C.at(index_b.at(i)).y;
    l = sqrt(pow(dx.x, 2) + pow(dx.y, 2));
    tmp_p.x = -dx.x / l;
    tmp_p.y = dx.y / l;
    contour_normal.push_back(tmp_p);
  }
  return contour_normal;
}



void snake_move_iteration(Mat internal_force_matrix, vector<Point> &C,
                          Mat &x, Mat &y, Mat u,
                          Mat v, double time_step = 1, double kappa = 10,
                          double delta = 0.01) {
  // kappa : external image field weight
  // delta: Balloon Force weight
  min_max(x,y, C,1, u);
  //TODO get u[x,y] for all x and y, v[x,y] for all x and y failed
  Mat fxq(x.size(),x.type());
  Mat fyq(x.size(),x.type());

  for(int i=0;i<x.cols;i++) {
    fxq.at<float>(i,0) = u.at<float>(x.at<int>(i,0), y.at<int>(i,0));
    fyq.at<float>(i,0) = v.at<float>(x.at<int>(i,0), y.at<int>(i,0));
  }

  cout << fxq << endl;

  x = internal_force_matrix * (time_step * x + fxq * kappa);
  y = internal_force_matrix * (time_step * x + fyq * kappa);

  min_max(x,y, C,1, u);

}



int main(int argc, char *argv[]) {
  for (int i = 0; i < argc; ++i)
    cout << argv[i] << endl;

  // calculate the gvf
  float mu = 0.2;
  int ITER = 50;
  // hyperparameter
  double alpha = 0.01;
  double beta = 0.01;
  double gamma = 1;

  // read the image
  Mat input_img = imread(argv[1], 0);

  // generate the contour, we initialize it as a circle using opencv function

  vector<vector<Point>> snake_coords;
  Mat contour_img = Mat::zeros(input_img.rows, input_img.cols, CV_8U);
  contour_initialize_1(150, 170, 120, contour_img, snake_coords);
  int num_points = snake_coords[0].size();
  vector<Point> C = snake_coords[0];
  /*cout << "************************" << endl;
  for(int i{0};i<C.size();i++)
      cout << C.at(i) << " ";
  cout << endl;*/
  Mat e_ext;
  e_ext = external_force_image(input_img, 0.04, 2, 0.01, 10);
  // namedWindow("e_ext", 0);
  // imshow("e_ext", e_ext);
  // waitKey(0);
  int sigma2 = 20;
  Mat image, fx, fy, fx_eq;

  GaussianBlur(e_ext, image, Size(3, 3), sigma2, sigma2);
  Sobel(image, fx, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
  Sobel(image, fy, CV_32F, 0, 1, 3);
  /*namedWindow("sobel x: y direction will be drawn", 0);
  imshow("sobel x: y direction will be drawn", fx);
  waitKey(0);
  namedWindow("sobel y: x direction will be drawn", 0);
  imshow("sobel y: x direction will be drawn", fy);
  waitKey(0);*/
  // normalizeMat(fx, fy);
  // equalizeHist(fx,fx_eq);
  Mat ux, uy; // gradient vector flow
  multiply(fx, -2 * sigma2 * sigma2, ux);
  multiply(fy, -2 * sigma2 * sigma2, uy);

  normalizeMat(ux, uy);
  /*namedWindow("ux_main", 0);
  imshow("ux_main", ux);
  waitKey(0);

  namedWindow("uy_main", 0);
  imshow("uy_main", uy);
  waitKey(0);
  cout << "Get into gradient vector field" << endl;
   */
  grad_vector_field(ux, uy, mu, ITER);
  /*
      namedWindow("ux after gvf", 0);
      imshow("ux after gvf", ux);
      waitKey(0);

      namedWindow("uy after gvf", 0);
      imshow("uy after gvf", uy);
      waitKey(0);
      normalizeMat(ux,uy);

      namedWindow("ux after normalization", 0);
      imshow("ux after normalization", ux);
      waitKey(0);

      namedWindow("uy after normalization", 0);
      imshow("uy after normalization", uy);
      waitKey(0);
      cout << "get out of gradient vector field" << endl;*/
  // visualization of the gradient vector flow
  Mat dem_img, snake_move, tmp;
  input_img.convertTo(tmp, 0);
  cvtColor(tmp, dem_img, CV_GRAY2RGB);
  dem_img.copyTo(snake_move);
  Scalar color(0, 255, 0);
  drawOptFlowMap(uy, ux, dem_img, 8, 8, color);
  namedWindow("Grad Field", 0);
  imshow("Grad Field", dem_img);
  waitKey(0);

  // calculate the internal force matrix
  Mat internal_force_matrix;

  snake_internal_force_matrix2d(0.01, 0.01, 1, internal_force_matrix,
                                num_points);
  // namedWindow("Internal force matrix", 0);
  // imshow("Internal force matrix", internal_force_matrix);
  // waitKey(0);

  // move the snake iteratively
  size_t snake_len = C.size();
  Mat x(1, snake_len, CV_32F);
  Mat y(1, snake_len, CV_32F);
  cout << x.size() << endl;
  float* x_ptr, *y_ptr;
  x_ptr = x.ptr<float>(0);
  y_ptr = y.ptr<float>(0);
  for (size_t i=0; i<snake_len; i++) {
    x_ptr[i] = C[i].x;
    y_ptr[i] = C[i].y;
  }

  cout << C.size() << endl;
  int max_iterations = 100;
  Mat dbg_img;
  cvtColor(tmp, dbg_img, CV_GRAY2RGB);

  cout << "Starting " << endl;
  // Find shape by moving snake on image
  namedWindow("Snake on Image", 0);

  Mat snake_move_orig = snake_move.clone();
  for (int i{0}; i < max_iterations; i++) {
    snake_move_orig.copyTo(snake_move);

    snake_move_iteration(internal_force_matrix, C, x,y,uy, ux, 1, 10, 0.01);

    cout << y.size() << endl;
    cout << x.size()<<endl;
    for (size_t j=0;  j<x.cols; j++) {

      snake_move.at<float>(floor(x.at<float>(0, j)), floor(y.at<float>(0, j))) = 255;
    }
    dispImage(snake_move, "Snake on Image", 50);
  }

  for (size_t j=0; j<x.cols; j++) {
    dem_img.at<Vec3b>(floor(y.at<float>(0, j)), floor(x.at<float>(0, j))) = 255;
  }

  return 0;
}
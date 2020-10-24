//
// Created by kit on 18.10.20.
//
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <stdio.h>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
using namespace std;
using namespace cv;

Mat im, edge_img;
int edge_threshold = 0;
//--Overloaded functions to display an image in a new window--//
void dispImage(Mat& img) {

    if(img.empty()) { //Read image and display after checking for image validity
        cout << "Error reading image file!";
        cin.ignore();
    } else {
        namedWindow("Image", 0);
        imshow("Image", img);
        waitKey();
    }
}


void dispImage(Mat& img, String windowName) {

    if(img.empty()) { //Read image and display after checking for image validity
        cout << "Error reading image File!";
        cin.ignore();
    }else {
        imshow(windowName, img);
        waitKey();
    }
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


void dispImage(Mat& img, String windowName, String error_msg) {

    if(img.empty()) { //Read image and display after checking for image validity
        cout << error_msg;
        cin.ignore();
    }else {
        namedWindow(windowName, 0);
        imshow(windowName, img);
        waitKey();
    }
}


void dispImage(Mat& img, String windowName, String error_msg, int delay) {

    if (img.empty()) { //Read image and display after checking for image validity
        cout << error_msg;
        cin.ignore();
    } else {
        namedWindow(windowName, 0);
        imshow(windowName, img);
        waitKey(delay);
    }
}

void applyEdgeThresh(int, void*) {

    Canny(im, edge_img, 0, edge_threshold, 3);
    imshow("Edge Image", edge_img);
}

void normalizeMat(Mat& fx, Mat& fy) {

    Mat mag(fx.size(), CV_32F);
    Mat prod_temp1, prod_temp2;

    //Magnitude Computation
    multiply(fx, fx, prod_temp1);
    multiply(fy, fy, prod_temp2);
    mag = prod_temp1 + prod_temp2;
    sqrt(mag, mag);

    fx = fx / mag; fy = fy / mag;
}

void gradVectorField(Mat& fx, Mat& fy, Mat& u, Mat& v, float mu, int ITER) {

    fx.copyTo(u);
    fy.copyTo(v);
    Mat gradX, gradY;
    Mat sqrMag, fx2, fy2;
    Mat u_term2, v_term2;

    multiply(fx, fx, fx2);
    multiply(fy, fy, fy2);
    sqrMag = fx2 + fy2;

    int dt;
    dt = 1;

    //Solve the differential equation iteratively//
    //Diffusion function being implemented is from the paper Snakes, Shapes and Gradient Vector FLow
    //by Chenyang Xu and Jerry L Prince
    // u = u + mu * del2(u) -(u - fx) * (fx2 + fy2);
    // v = v + mu * del2(v) - (v - fy) * (fx2 + fy2);
    for(int i = 0; i < ITER; i++) {
        Laplacian(u, gradX, CV_32F, 1, BORDER_REFLECT);
        Laplacian(v, gradY, CV_32F, 1, BORDER_REFLECT);

        multiply((u- fx), sqrMag, u_term2);
        multiply((v- fy), sqrMag, v_term2);

        u = u + dt * (mu * gradX - u_term2) ;
        v = v + dt * (mu * gradY - v_term2) ;
        namedWindow("11", 0);
        imshow("11", u);
        waitKey(0);
        namedWindow("22", 0);
        imshow("22", v);
        waitKey(0);
    }



    namedWindow("u Image", 0);
    dispImage(u,"u Image");
    waitKey(0);

    normalizeMat(u, v);
    namedWindow("u Image", 0);
    dispImage(u,"u Image");
    waitKey(0);
}
Mat external_force_image(Mat img,
                         const float w_line,
                         const float w_edge,
                         const float w_term,
                         const float sigma)
{
    Mat Ix, Iy, Ixx, Iyy, Ixy;
    // Gaussian blur
    Mat image;
    GaussianBlur(img,image,Size(3,3),1,1);
    image.convertTo(image, CV_32F);

    // calculate the image first derivative w.r.t. x, y as well as the second derivative w.r.t xx,yy,xy
    Sobel(image, Ix, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
    Sobel(image, Iy, CV_32F, 0, 1, 3);
    Sobel(Ix, Ixx, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
    Sobel(Iy, Iyy, CV_32F, 0, 1, 3);
    Sobel(Ix, Ixy, CV_32F, 0,1,3);

    Mat e_line, e_edge, e_term, e_extern;
    GaussianBlur(image,e_line,Size(3,3),1,1);
    // Calculate (Iyy.*Ix.*Ix - 2*Ixy.*Ix.*Iy + Ixx.*Iy.*Iy)./ ((1+Ix.*Ix+Iy.*Iy)**1.5)
    // tmp1 = Iyy.*Ix.*Ix
    // tmp2 = 2*Ixy.*Ix.*Iy
    // tmp3 = Ixx.*Iy.*Iy
    // tmp4 = (1+Ix.*Ix+Iy.*Iy)
    Mat tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
    multiply(Ix,Ix,tmp1);
    multiply(Ix,Ix,tmp4);
    multiply(tmp1, Iyy, tmp1);
    multiply(Ix, Iy, tmp2);
    multiply(tmp2, -2*Ixy, tmp2);
    multiply(Iy, Iy, tmp3);
    multiply(Iy, Iy, tmp5);
    multiply(tmp3, Ixx, tmp3);

    e_term = tmp1 + tmp2 + tmp3;

    // cout << one_matrix << endl;
    Mat tes = tmp5 + tmp4 +1 ;
    // cout << tes << endl;

    pow(tmp5 + tmp4 + 1,1.5,tmp6);

    divide(e_term,tmp6,e_term);

    sqrt((tmp5 + tmp4),e_edge);

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

void grad_vector_field(Mat &u, Mat &v, float mu, int ITER)
{

    Mat sqrMag, u2, v2;
    Mat u0,v0;
    u.copyTo(u0);
    v.copyTo(v0);
    Mat u_term2, v_term2;
    Mat gradX,gradY;
    // square magnitude of force field
    multiply(u, u, u2);
    multiply(v, v, v2);
    sqrMag = u2 + v2;


    // cout << sqrMag << endl;
    int dt;
    dt = 1;

    //Solve the differential equation iteratively//
    //Diffusion function being implemented is from the paper Snakes, Shapes and Gradient Vector FLow
    //by Chenyang Xu and Jerry L Prince
    // u = u + mu * del2(u) -(u - fx) * (fx2 + fy2);
    // v = v + mu * del2(v) - (v - fy) * (fx2 + fy2);
    Mat img = imread("/home/kit/CLionProjects/Snake_active_contour/build/testimage.png",0);
    double sigma = 20;
    for(int i = 0; i < ITER; i++) {
        GaussianBlur(u,u,Size(3,3),sigma,sigma);
        GaussianBlur(v,v,Size(3,3),sigma,sigma);
        Laplacian(u, gradX, CV_32F, 1, BORDER_REFLECT);
        Laplacian(v, gradY, CV_32F, 1, BORDER_REFLECT);
        Mat u_term2, v_term2;
        multiply(sqrMag, (u-u0), u_term2);
        multiply(sqrMag, (v-v0), v_term2);
        u = u + dt * (mu *gradX - u_term2);
        v = v + dt * (mu *gradY - v_term2);
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
void contour_initialize_1(int center_x, int center_y, int radius, Mat& contour_img, vector<vector<Point>> &snake_coords) {

    Point center(center_x, center_y);
    circle(contour_img, center, radius, 255, 1, 8);

    findContours(contour_img, snake_coords, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
}
void drawOptFlowMap(Mat &fx, Mat &fy, Mat& cflowmap, int step,
                    double scaleFactor, Scalar& color) {
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step) {
            Point2f fxy;
            fxy.y = fy.at<float>(y, x);
            fxy.x = fx.at<float>(y, x);

            if(fxy.x !=0 || fxy.y != 0) {
                line(cflowmap, Point(x,y),
                     Point(cvRound(x+(fxy.x)*scaleFactor),
                           cvRound(y+(fxy.y)*scaleFactor)),
                     color);
            }
            circle(cflowmap, Point(x,y), 1, color, -1);
        }
}


int interp2d(const cv::Mat& xs,    // grid locations - x coords
             const cv::Mat& ys,    // grid locations - y coords
             const cv::Mat& fs,    // function values at grid locations
             cv::Mat& fq) {  // function query points
    int xs_len = xs.cols;
    fq.create(xs.size(), xs.type());

    // Get grid coordinates surrounding snake location
    /*------------*
      |  |  |  |  |
      *--*--*--*--*
      |  |  |  |  |
      *--*--*--*--*
      |  |  |  |  |
      *--*--*--*--*
      */
    Mat x1(xs.size(), xs.type());
    Mat x2(xs.size(), xs.type());
    Mat y1(ys.size(), ys.type());
    Mat y2(ys.size(), ys.type());

    // Access opencv Mat elements using pointers
    const float* xs_ptr  = xs.ptr<float>(0);
    const float* ys_ptr  = ys.ptr<float>(0);
    float* x1_ptr = x1.ptr<float>(0);
    float* x2_ptr = x2.ptr<float>(0);
    float* y1_ptr = y1.ptr<float>(0);
    float* y2_ptr = y2.ptr<float>(0);

    for (size_t i=0; i<xs_len; i++) {
        x1_ptr[i] = floor(xs_ptr[i]);
        x2_ptr[i] = ceil(xs_ptr[i]);
        y1_ptr[i] = floor(ys_ptr[i]);
        y2_ptr[i] = ceil(ys_ptr[i]);
    }

    Mat xdiff1, xdiff2, xdiff3, ydiff1, ydiff2, ydiff3;
    subtract(x2, xs, xdiff1);
    subtract(xs, x1, xdiff2);
    subtract(x2, x1, xdiff3);
    subtract(y2, ys, ydiff1);
    subtract(ys, y1, ydiff2);
    subtract(y2, y1, ydiff3);

    // Formula used to compute BLT:
    // fq(x, y) = 1/((x2-x1) * (y2-y1))*[((x2-x) * (y2-y) * fx(x1, y1))
    //            +((x-x1) * (y2-y) * fx(x2, y1))
    //            +((x2-x) * (y-y1) * fx(x1, y2))
    //            +((x-x1) * (y-y1) * fx(x2, y2))]

    // Calculate the numerator and denominator components independently
    Mat nr1, nr2, nr3, nr4;
    Mat dr;
    multiply(xdiff1, ydiff1, nr1);    //nr1 = xdiff1 * ydiff1;
    multiply(xdiff2, ydiff1, nr2);   //nr2 = xdiff2 * ydiff1;
    multiply(xdiff1, ydiff2, nr3);   //nr3 = xdiff1 * ydiff2;
    multiply(xdiff2, ydiff2, nr4);    //nr4 = xdiff2 * ydiff2;
    multiply(xdiff3, ydiff3, dr);    //dr = xdiff3 * ydiff3;

    // Pointers to newly created matrices
    float* xdiff2_ptr = xdiff2.ptr<float>(0);
    float* xdiff3_ptr = xdiff3.ptr<float>(0);
    float* ydiff2_ptr = ydiff2.ptr<float>(0);
    float* ydiff3_ptr = ydiff3.ptr<float>(0);
    float* nr1_ptr = nr1.ptr<float>(0);
    float* nr2_ptr = nr2.ptr<float>(0);
    float* nr3_ptr = nr3.ptr<float>(0);
    float* nr4_ptr = nr4.ptr<float>(0);
    float* dr_ptr  = dr.ptr<float>(0);
    float* fq_ptr  = fq.ptr<float>(0);


    // Compute function value at query points
    for(int i=0; i<xs_len; i++) {

        if ((xdiff3_ptr[i] == 0) && (ydiff3_ptr[i] == 0)) {
            // If point is located at a valid location, copy function value at that point directly
            fq_ptr[i] = fs.at<float>(int(xs_ptr[i]), int(ys_ptr[i]));

        } else if ((xdiff3_ptr[i] != 0) && (ydiff3_ptr[i] == 0)) {
            // For Points located on same grid line along y make it a linear interpolation along x
            fq_ptr[i] =  (fs.at<float>(int(x2_ptr[i]), int(y2_ptr[i]))
                          - fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i])))
                         * xdiff2_ptr[i] / xdiff3_ptr[i]
                         + fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i]));

        } else if((xdiff3.at<float>(0, i) == 0) && (ydiff3.at<float>(0, i) != 0)) {
            // For Points located on same grid line along x make it a linear interpolation along y
            fq_ptr[i] =  (fs.at<float>(int(x2_ptr[i]), int(y2_ptr[i]))
                          - fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i])))
                         * ydiff2_ptr[i] / ydiff3_ptr[i]
                         + fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i]));

        } else {
            // For points located at positions in between grid points, use formula for BLT
            fq_ptr[i] =  (nr1_ptr[i] * fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i]))
                          + nr2_ptr[i] * fs.at<float>(int(x2_ptr[i]), int(y1_ptr[i]))
                          + nr3_ptr[i] * fs.at<float>(int(x1_ptr[i]), int(y2_ptr[i]))
                          + nr4_ptr[i] * fs.at<float>(int(x2_ptr[i]), int(y2_ptr[i]))) / dr_ptr[i];
        }
    }

    xdiff1.release();
    xdiff2.release();
    xdiff3.release();
    ydiff1.release();
    ydiff2.release();
    ydiff3.release();
    nr1.release();
    nr2.release();
    nr3.release();
    nr4.release();
    dr.release();

    return 0;
}


void get_contour_normal(vector<Point> C, Mat &x, Mat &y)
{
    int a = 4; // use the n'th neighbour to calculate the normal
    int n = C.size();

    vector<int> index_f, index_b;
    // vector<Point> contour_normal;
    for(int i{1}; i<n+1; i++)
    {
        if(i + a > n)
            index_f.push_back(i+a-n);
        else
            index_f.push_back(i+a);

        if(i - a < 1)
            index_b.push_back(i-a+n);
        else
            index_b.push_back(i-a);
    }

    // for(int i{0}; i<index_f.size();++i)
    //   cout << index_f.at(i) << " ";
    //cout << endl;
    Point dx;
    float l =0.0;
    // Point tmp_p;
    for(int i=0;i<n;i++)
    {
        dx.x = C.at(index_f.at(i)-1).x - C.at(index_b.at(i)-1).x;
        dx.y = C.at(index_f.at(i)-1).y - C.at(index_b.at(i)-1).y;
        l = sqrt(pow(dx.x,2) + pow(dx.y,2));


        x.at<float>(i) = - dx.x / l;
        y.at<float>(i) = dx.y / l;
        // contour_normal.push_back(tmp_p);
    }


}

int deformSnake(vector<Point> C,
                cv::Mat& x, cv::Mat& y,
                const cv::Mat& ext_fx,
                const cv::Mat& ext_fy,
                const float& alpha,
                const float& beta,
                const float& gamma,
                const float& kappa,
                const float& delta,
                const int& iter) {

    size_t snake_len = x.cols;

    float a = gamma * (2*alpha + 6*beta) + 1;
    float b = gamma * (-alpha - 4*beta);
    float c = beta  * gamma;

    Mat ldiag(snake_len, 1, CV_32F, Scalar(a));
    Mat pd_mat = Mat::diag(ldiag);

    size_t last_elem = snake_len - 1;
    float* pd_ptr; // internal force matrix

    // Populate elements of the NxN penta-diagonal matrix
    // Where N, is the length of snake
    for (size_t i=0; i<snake_len; i++) {
        pd_ptr = pd_mat.ptr<float>(i);
        if (i == 0) {
            pd_ptr[i+1] = b;
            pd_ptr[i+2] = c;
            pd_ptr[last_elem-1] = c;
            pd_ptr[last_elem] = b;
        } else if (i == 1) {
            pd_ptr[i-1] = b;
            pd_ptr[i+1] = b;
            pd_ptr[i+2] = c;
            pd_ptr[last_elem] = c;
        } else if ((i > 1) && (i < last_elem-1)) {
            pd_ptr[i-2] = c;
            pd_ptr[i-1] = b;
            pd_ptr[i+1] = b;
            pd_ptr[i+2] = c;
        } else if (i == last_elem-1) {
            pd_ptr[i-1] = b;
            pd_ptr[i+1] = b;
            pd_ptr[i-2] = c;
            pd_ptr[0] = c;
        } else {
            pd_ptr[i-1] = b;
            pd_ptr[i-2] = c;
            pd_ptr[0] = b;
            pd_ptr[1] = c;
        }
    }

    // Compute matrix inverse: (I-gamma*pd_mat)^(-1)
    Mat i_gamma_pd_inv = pd_mat.inv();
    Mat fxq, fyq,fxq_ballon,fyq_ballon;

    // Deform snake for the specified no of iterations
    // Update snake coordinates based on value of external constraint function
    // at each coordinate of x and y
    Mat xx(1, snake_len, CV_32F);
    Mat yy(1, snake_len, CV_32F);

    for (size_t i=0; i<1; i++) {

        // Get x and y components of external constraint force
        // at snake location

        interp2d(x, y, ext_fx, fxq);
        interp2d(x, y, ext_fy, fyq);
        // get_contour_normal(C, xx,yy);
        // interp2d(xx, yy, ext_fx, fxq_ballon);
        // interp2d(xx, yy, ext_fy, fyq_ballon);
        // x = (x + gamma * kappa + fxq) * i_gamma_pd_inv;

        x = (x + kappa*gamma + fxq) * i_gamma_pd_inv;
        y = (y + kappa*gamma + fyq) * i_gamma_pd_inv;
    }



    return 0;
}



int interp1d(const cv::Mat& x, const cv::Mat& fx, const cv::Mat& xq, cv::Mat& fq) {
    //Mat fq(xq.size(), xq.type());
    Mat mask(x.size(), CV_8U, Scalar(1));
    Mat diff_mat(x.size(), x.type());
    double min_val;
    Point nbr_00, nbr_01;

    int arr_len = xq.cols;
    const float* xq_ptr = xq.ptr<float>(0);
    float* fq_ptr = fq.ptr<float>(0);

    for (size_t i=0; i<arr_len; i++) {
        diff_mat = abs(xq_ptr[i] - x);

        // Locate the 2 closest neighbors to query point in sample point array
        minMaxLoc(diff_mat, &min_val, 0, &nbr_00, 0, mask);
        mask.at<uchar>(nbr_00.y, nbr_00.x) = (uchar)0;
        minMaxLoc(diff_mat, &min_val, 0, &nbr_01, 0, mask);
        mask.at<uchar>(nbr_00.y, nbr_00.x) = (uchar)1;

        float thresh = 0.1; // Set heuristically - play around for better results
        if (abs(x.at<float>(nbr_01.y, nbr_01.x) - x.at<float>(nbr_00.y, nbr_00.x)) < thresh) {
            //if ((nbr_01.x == (x.cols - 1) || (nbr_00.x == (x.cols - 1`))) {
            //fq_ptr[i] = fx.at<float>(0, 0)r;
            //} else {
            // If 2 neighbors are too close, take function value from one of the existing pts
            fq_ptr[i] = fx.at<float>(nbr_00.y, nbr_00.x);
            //}

            //if (i == arr_len -1) {
            //cout << "Inside If" << endl;
            //cout << x.at<float>(nbr_01.y, nbr_01.x) << endl;
            //cout << x.at<float>(nbr_00.y, nbr_00.x) << endl;
            //cout << fq_ptr[i] << endl;
            //}
        } else {
            // If not apply 1D interpolation function
            fq_ptr[i] = (fx.at<float>(nbr_01.y, nbr_01.x) - fx.at<float>(nbr_00.y, nbr_00.x))
                        * (xq_ptr[i] - x.at<float>(nbr_00.y, nbr_00.x))
                        / (x.at<float>(nbr_01.y, nbr_01.x) - x.at<float>(nbr_00.y, nbr_00.x))
                        + fx.at<float>(nbr_00.y, nbr_00.x);
        }
    }
    return 0;
}


int calcDist(const cv::Mat& x, cv::Mat& dx) {

    if (x.cols < 5) {
        cout << "Error! Contour too small! " << endl;
        return -1;
    }

    const float* x_ptr = x.ptr<float>(0);
    float* dx_ptr = dx.ptr<float>(0);

    size_t x_len = x.cols;
    for (size_t i=0; i<x_len; i++) {
        dx_ptr[i] = abs(x_ptr[i] - x_ptr[i+1]);
    }

    // for closed contour, use first and last point to compute dx[0]
    dx_ptr[x_len-1] = abs(x_ptr[0] - x_ptr[x_len-1]);
    return 0;
}


//--Function to return values of a 1D function at specific query points based on sample function values--//
//--using Linear Interpolation//
Mat interp1d(Mat x, Mat fx, Mat xq) {

//--x: Independent Variable; f(x): Dependent Variable;
//--xq: Query points; fq(xq) is calculated using this function--//
    Mat fq(xq.size(), xq.type());

    Mat Mask(x.size(), CV_8U, Scalar(1));			//Mask image used in finding second minimum point
    double min_val;
    Point min_pt1, min_pt2;
    Mat diff_mat(x.size(), x.type());				//Matrix to hold distance between xq and x values

    int col_count = xq.cols;	//Length of vector containing query points
    float* xq_ptr, *fq_ptr;
    xq_ptr = xq.ptr<float>(0);
    fq_ptr = fq.ptr<float>(0);

    for(int i = 0; i <  col_count; i++) {

        Mat ones_vector = Mat::ones(x.size(), x.type()); //Temporary vector used during distance value computation
        diff_mat = abs(xq_ptr[i] - x); //Calcualte distance between query point and all x's
        //multiply(ones_vector, xq_ptr[i], ones_vector);
        //absdiff(ones_vector,  x, diff_mat);

        //Find the two closest neighbours to xq and interpolate (or extrapolate)
        minMaxLoc(diff_mat, &min_val, 0, &min_pt1, 0, Mask);
        Mask.at<uchar>(min_pt1.y, min_pt1.x) = (uchar)0;
        minMaxLoc(diff_mat, &min_val, 0, &min_pt2, 0, Mask);
        Mask.at<uchar>(min_pt1.y, min_pt1.x) = (uchar)1;

        if((x.at<float>(min_pt2.y, min_pt2.x) - x.at<float>(min_pt1.y, min_pt1.x)) < 0.1) {

            if(min_pt1.x == (x.cols - 1))
                fq_ptr[i] = fx.at<float>(0, 0);
            else
                fq_ptr[i] = fx.at<float>(min_pt1.y, min_pt1.x);

        }else
            fq_ptr[i] = (fx.at<float>(min_pt2.y, min_pt2.x) - fx.at<float>(min_pt1.y, min_pt1.x))
                        * (xq_ptr[i] - x.at<float>(min_pt1.y, min_pt1.x)) / (x.at<float>(min_pt2.y, min_pt2.x) - x.at<float>(min_pt1.y, min_pt1.x))
                        + fx.at<float>(min_pt1.y, min_pt1.x);

        //cout << "Interpolated x: " << fq.at<float>(i, 0) << ", ";

    }

    return fq;
}

Mat getInterpIndex(const Mat &IDX) {

    int y_col_count = 2 * IDX.cols;
    Mat y(1, y_col_count, CV_32F);

    //Create indexes inbetween all points (this will be the index of x/y coordinates)
    for (int i = 0; i < y_col_count; i++) {
        y.at<float>(0, i) = 0.5 * i;
    }

    //Create new array to hold index points temporarily
    float y_new[3000];
    int count = 0;

    for (int i = 0; i < IDX.cols; i++) { // Iterate over the IDX array and keep new points
        // (Eg: point 1.5 is retained if d(1, 2) > dmax)
        // where distance between 2 points is greater than dmax
        y_new[count++] = y.at<float>(0, 2*i + 0);
        if (IDX.at<int>(0, i) == 1)	{ //odd locations in y hold decimal values, even locations the whole numbers
            y_new[count++] = y.at<float>(0, 2*i + 1);
        }
    }

    Mat y1(1, count, CV_32F); //Create new matrix and copy values(indices) from array to matrix
    for(int i = 0; i < count; i++) {
        y1.at<float>(0, i) = y_new[i];
    }
    return y1;
}

int interpolateSnake(const float dmin, const float dmax, cv::Mat& x, Mat& y, Mat& img) {
    Mat dx, dy, d;
    int j;
    dx.create(x.size(), CV_32F);
    dy.create(y.size(), CV_32F);
    d.create(y.size(), CV_32F);
    calcDist(x, dx);
    calcDist(y, dy);
    //d = dx+dy;
    add(dx, dy, d);
    dx.release();
    dy.release();

    //-- Find and eliminate points located closer than dmin
    Mat x_dmin(x.size(), x.type());
    Mat y_dmin(y.size(), y.type());
    float* x_dmin_ptr, *y_dmin_ptr, *x_ptr, *y_ptr, *d_ptr;
    x_ptr = x.ptr<float>(0);
    y_ptr = y.ptr<float>(0);
    x_dmin_ptr = x_dmin.ptr<float>(0);
    y_dmin_ptr = y_dmin.ptr<float>(0);
    d_ptr = d.ptr<float>(0);
    size_t loop_term = x.cols;
    int valid_pt_count = 0;

    // Go through dist vector and flag points that are too close
    for (size_t i=0; i<loop_term; i++) {
        if (d_ptr[i] < dmin) {
            x_dmin_ptr[i] = float(-1);
            y_dmin_ptr[i] = float(-1);
        } else {
            x_dmin_ptr[i] = x_ptr[i];
            y_dmin_ptr[i] = y_ptr[i];
            valid_pt_count++;
        }
    }

    cout << "Image data" << img.size() << endl;
    for (size_t j=0; j<x.cols; j++) {
        img.at<Vec3b>(floor(y.at<float>(0, j)), floor(x.at<float>(0, j))) = 255;
    }
    //namedWindow("Pts", 0);
    //imshow("Pts", img);
    //waitKey();

    // Create a new matrix and copy only valid points from original array
    Mat x_new (1, valid_pt_count, CV_32F);
    Mat y_new (1, valid_pt_count, CV_32F);

    float* x_new_ptr = x_new.ptr<float>(0);
    float* y_new_ptr = y_new.ptr<float>(0);
    int c = 0;

    for (size_t i=0; i<loop_term; i++) {
        if (x_dmin_ptr[i] == -1) {
            continue;
        } else {
            x_new_ptr[c] = x_ptr[i];
            y_new_ptr[c++] = y_ptr[i];
        }
    }

    // Find points that are too far away [d > dmax]
    double max_d;
    Mat dmax_indicator, ind_var, q_pts;
    dx.create(x_new.size(), CV_32F);
    dy.create(y_new.size(), CV_32F);
    calcDist(x_new, dx);
    calcDist(y_new, dy);
    d = dx + dy;
    d_ptr = d.ptr<float>(0);
    minMaxLoc(d, 0, &max_d, 0, 0);
    Mat xq_new, yq_new;
    float* ind_var_ptr;
    int max_iter = 100;
    int iter_cnt = 0;

    // Check and interpolate until all 'gaps' in curve are filled
    //namedWindow("Pts", 0);
    while (max_d > dmax) {
        cout << "Max Val " << max_d << "/" << dmax << endl;
        dmax_indicator.create(x_new.size(), CV_32S);
        int* dmax_ptr = dmax_indicator.ptr<int>(0);
        loop_term = dmax_indicator.cols;
        ind_var.create(x_new.size(), x_new.type());

        for (size_t i=0; i<loop_term; i++) {
            if (d_ptr[i] > dmax)
                dmax_ptr[i] = 1;
            else
                dmax_ptr[i] = 0;
        }

        // indices for x
        ind_var.create(x_new.size(), x_new.type());
        ind_var_ptr = ind_var.ptr<float>(0);

        size_t ind_var_siz = xq_new.cols;
        for (size_t i=0; i<loop_term; i++) {
            ind_var_ptr[i] = float(i);
        }

        q_pts = getInterpIndex(dmax_indicator);

        xq_new.create(q_pts.size(), CV_32F);
        yq_new.create(q_pts.size(), CV_32F);
        interp1d(ind_var, x_new, q_pts, xq_new);
        interp1d(ind_var, y_new, q_pts, yq_new);
        x_new = xq_new; y_new = yq_new;

        // Debugging code begin
        if (x_new.cols < 10) {
            cout << "Error! Contour too small!" << endl;
            break;
        }

        dx.create(x_new.size(), CV_32F);
        dy.create(y_new.size(), CV_32F);
        calcDist(x_new, dx);
        calcDist(y_new, dy);
        d = dx + dy;
        d_ptr = d.ptr<float>(0);
        minMaxLoc(d, 0, &max_d, 0, 0);

        // Exit loop if max iter is reached
        iter_cnt++;
        if (iter_cnt > max_iter) {
            cout << "Max iter reached " << endl;
            break;
        }
    }
    x = x_new;
    y = y_new;
    cout << "Snake Interpolation Done! " << endl;
    return 0;
}

int main(int argc, char* argv[]) {

    if (argc != 2) {
        cout << "Error! Syntax is ./build/snake <img_name>" << endl;
        return -1;
    }

    // parameters
    int center_x = 150;
    int center_y = 170;
    int radius = 120;
    // calculate the gvf
    float mu = 0.2;
    int ITER = 50;
    // hyperparameter
    double alpha = 0.01;
    double beta = 0.01;
    double gamma = 1;

    // read the image
    Mat input_img = imread(argv[1],0);


    // generate the contour, we initialize it as a circle using opencv function

    vector<vector<Point>> snake_coords;
    contour_img = Mat::zeros(input_img.rows, input_img.cols, CV_8U);
    contour_initialize_1(center_x, center_y, radius, contour_img,snake_coords);
    int num_points = snake_coords[0].size();
    vector<Point> C = snake_coords[0];
    /*cout << "************************" << endl;
    for(int i{0};i<C.size();i++)
        cout << C.at(i) << " ";
    cout << endl;*/
    Mat e_ext;
    e_ext = (input_img, 0.04,2,0.01,10);


    int sigma2 = 20;
    Mat image, fx, fy, fx_eq;

    GaussianBlur(input_img,image,Size(3,3),sigma2,sigma2);
    Sobel(image, fx, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
    Sobel(image, fy, CV_32F, 0, 1, 3);
    /*namedWindow("sobel x: y direction will be drawn", 0);
    imshow("sobel x: y direction will be drawn", fx);
    waitKey(0);
    namedWindow("sobel y: x direction will be drawn", 0);
    imshow("sobel y: x direction will be drawn", fy);
    waitKey(0);*/
    normalizeMat(fx, fy);
    // equalizeHist(fx,fx_eq);
    Mat ux,uy; // gradient vector flow
    multiply(fx,-2*sigma2*sigma2,ux);
    multiply(fy,-2*sigma2*sigma2,uy);

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
    Mat dem_img, snake_move,tmp;
    input_img.convertTo(tmp,0);
    cvtColor(tmp, dem_img, CV_GRAY2RGB);
    dem_img.copyTo(snake_move);
    Scalar color(0, 255, 0);
    drawOptFlowMap(uy, ux, dem_img, 8, 8, color);
    namedWindow("Grad Field", 0);
    imshow("Grad Field", dem_img);
    waitKey(0);


    // Initialize Snakes
    cout << "Now please initialize the snakes" << endl;
    contour_img = Mat::zeros(im.rows, im.cols, CV_8U);
    // TODO
    Point center(150, 170);

    circle(contour_img, center, 100, 255, 1, 8);

    findContours(contour_img, snake_coords, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    size_t slen = snake_coords[0].size();
    Mat x(1, slen, CV_32F);
    Mat y(1, slen, CV_32F);
    cout << x.size() << endl;
    float* x_ptr, *y_ptr;
    x_ptr = x.ptr<float>(0);
    y_ptr = y.ptr<float>(0);
    for (size_t i=0; i<slen; i++) {
        x_ptr[i] = snake_coords[0][i].x;
        y_ptr[i] = snake_coords[0][i].y;
    }



    cout << "Starting " << endl;
    // Find shape by moving snake on image
    namedWindow("Snake on Image", 0);
    Mat smove_orig = smove.clone();
    for (size_t i=0; i<250; i++) {
        smove_orig.copyTo(smove);
        // Mat internal_force_inverse;
        deformSnake(snake_coords[0],y, x, uy, ux, 0.01, 0.01, 1, 10, 0.01, 15);
        // add ballon force
        interpolateSnake(1, 2, x, y, dbg_img);




        //x += xx;
        //y += yy;
        // x = x * internal_force_inverse;
        // y = y * internal_force_inverse;
        // interpolateSnake(1, 2, x, y, dbg_img);
        for (size_t j=0; j<x.cols; j++) {
            smove.at<Vec3b>(floor(y.at<float>(0, j)), floor(x.at<float>(0, j))) = 255;
        }
        dispImage(smove, "Snake on Image", 50);
        //
    }
    imwrite("snake.jpg", smove);
    for (size_t j=0; j<x.cols; j++) {
        dem_img.at<Vec3b>(floor(y.at<float>(0, j)), floor(x.at<float>(0, j))) = 255;
    }
    return 0;
}
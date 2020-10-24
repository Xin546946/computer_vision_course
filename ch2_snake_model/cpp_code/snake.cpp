//
// Created by kit on 18.10.20.
//

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
using namespace std;
using namespace cv;



int main(int argc, char* argv[])
{
    Mat a = (Mat_<double>(3,3) << 1,2,3,4,5,6,7,8,9);
    Mat m ;
    multiply(a,-3*3,m);
    cout << m << endl;
    Mat I,Igaussian;
    I = imread("/home/kit/CLionProjects/Snake_active_contour/build/lena.png",0);
    namedWindow("image", WINDOW_AUTOSIZE);
    imshow("image", I);
    waitKey(0);
    GaussianBlur(I,Igaussian,Size(9,9),10,10);
    namedWindow("Igaussian", WINDOW_AUTOSIZE);
    imshow("Igaussian", Igaussian);
    waitKey(0);

    Mat dst;
    equalizeHist( I, dst );

    imshow( "Equalized Image", dst );
    waitKey(0);
   return 0;
}
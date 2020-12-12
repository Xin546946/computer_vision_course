#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

using namespace std;
using namespace cv;

int main() {
  Mat image1, image2;
  vector<Point2f> point1, point2, pointCopy;
  vector<uchar> status;
  vector<float> err;

  VideoCapture video(0);
  video >> image1;
  Mat image1Gray, image2Gray;
  cvtColor(image1, image1Gray, CV_RGB2GRAY);
  goodFeaturesToTrack(image1Gray, point1, 100, 0.01, 10, Mat());
  pointCopy = point1;
  for (int i = 0; i < point1.size(); i++) //绘制特征点位
  {
    circle(image1, point1[i], 1, Scalar(0, 0, 255), 2);
  }
  namedWindow("光流特征图");
  while (true) {
    video >> image2;
    if (waitKey(33) == ' ') //按下空格选择当前画面作为标定图像
    {
      break;
    }
    cvtColor(image2, image2Gray, CV_RGB2GRAY);
    calcOpticalFlowPyrLK(image1Gray, image2Gray, point1, point2, status, err,
                         Size(50, 50), 3); // LK金字塔

    for (int i = 0; i < point2.size(); i++) {

      circle(image2, point2[i], 1, Scalar(0, 0, 255), 2);
      line(image2, pointCopy[i], point2[i], Scalar(255, 0, 0), 2);
      std::cout << "Status: " << int(status[i]) << '\n';
    }

    imshow("光流特征图", image2);
    swap(point1, point2);
    image1Gray = image2Gray.clone();
  }
  return 0;
}
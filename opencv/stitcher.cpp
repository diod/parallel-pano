#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
#include<sys/time.h>
#include<vector>
#include "opencv2/stitching/stitcher.hpp"

using namespace std;
using namespace cv;
 
int main(int argc, char *argv[]) {
  cv::namedWindow("Cam", CV_WINDOW_NORMAL);
  cv::resizeWindow("Cam", 525,700);

  cv::VideoCapture cap("/export/video-1458503897.avi");
  
  if ( !cap.isOpened() )  // if not success, exit program
    {
       printf("Cannot open the video file\n");
       return -1;
    }

  vector<Mat> imgs;
    
  for(int i=0; i<80;i++) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      printf("Cannot read frame\n");
      break;
    }

    cv::Mat rot;

/* 90CW */
    cv::transpose(frame,rot);
    cv::flip(rot,rot,1);

    imshow("Cam", rot);

    if (i>30 && i%6==3) {
      imgs.push_back(rot.clone());
    }

    char k = (char)cv::waitKey(1);
    if (k==27) break;
    
    printf("frame: %d\n",i);
  }  

  printf("All loaded: %d\n", imgs.size());    

  cv::namedWindow("Pano", CV_WINDOW_NORMAL);
  cv::Mat pano;
 
  cv::Stitcher stitcher = cv::Stitcher::createDefault(false);
  Stitcher::Status status = stitcher.stitch(imgs,pano);
  
  if (status != Stitcher::OK) {
    printf("Error stitching images!\n");
    return -1;
  }

  printf("Stitching done: %d x %d\n", pano.cols, pano.rows);
  

  imwrite("output.png", pano);
  return 0;

}
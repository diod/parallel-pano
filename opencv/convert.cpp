#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
#include<sys/time.h>
#include<vector>
#include "opencv2/stitching/stitcher.hpp"

using namespace std;
using namespace cv;
 
int main(int argc, char *argv[]) {

  if (argc==1) {
    printf("Need avi file name for conversion\n");
    return -1;
  }  

  cv::VideoCapture cap(argv[1]);
  if ( !cap.isOpened() )  // if not success, exit program
    {
       printf("Cannot open the video file\n");
       return -1;
    }

  cv::namedWindow("Cam", CV_WINDOW_NORMAL);
  cv::resizeWindow("Cam", 525,700);


  for(int i=0; i<1024;i++) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      printf("Cannot read frame: exiting\n");
      break;
    }

    cv::Mat rot;

/* 90CW */
    cv::transpose(frame,rot);
    cv::flip(rot,rot,1);

    imshow("Cam", rot);

    if (i>20 && i%13==1) {
      char fname[1024];
      char *fn = &(fname[0]);
      
      sprintf(fn, "src/%04d.png", i);
      printf("save: ");
      
      imwrite(fn, rot);
    }

    char k = (char)cv::waitKey(1);
    if (k==27) break;
    
    printf("frame: %d\n",i);
  }  
  return 0;
/*

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
*/
}
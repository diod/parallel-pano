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

  int frame_width  = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  char filename[1024];
  char *fn = &(filename[0]);
  sprintf(fn, "/mnt/conv-%s", argv[1]);
  
  cv::VideoWriter video(fn, CV_FOURCC('M','J','P','G'),25,cv::Size(frame_height, frame_width),true);
  printf("Recorder: %dx%d\n",frame_width, frame_height);



  cv::namedWindow("Cam", CV_WINDOW_NORMAL);
  cv::resizeWindow("Cam", 640,480);

  int i=0;
  for(;;i++) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      printf("Cannot read frame: exiting\n");
      break;
    }
//    if (i<1000) continue;

    cv::Mat rot;

/* 90CW */
    cv::transpose(frame,rot);
    cv::flip(rot,rot,1);

    if (i>430) {
        video.write(rot);
        printf("w ");
    }
    imshow("Cam", rot);

    if (i==1700) break;

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
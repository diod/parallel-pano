#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
#include<sys/time.h>

volatile uint32_t compass_value = -1;

int main(int argc, char *argv[]) {
  cv::Mat frame;
  cv::namedWindow("Cam", CV_WINDOW_NORMAL);
  cv::resizeWindow("Cam", 525,700);
  
  cv::VideoCapture cap(1);

  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
  
  int frame_width  = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  struct timeval tm;
  gettimeofday(&tm, NULL);
  
  char filename[1024];
  sprintf(&(filename[0]), "/export/video-%ld.avi", tm.tv_sec);
  
  cv::VideoWriter video(filename, CV_FOURCC('M','J','P','G'),25,cv::Size(frame_width, frame_height),true);
  printf("Recorder: %dx%d\n",frame_width, frame_height);

  cv::Point2f src_center(frame_width/2.0F, frame_height/2.0F);
  cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, 90, 1.0);

  cv::Mat rot;
  
  long int i=0;
  for(;;i++){
    cap >> frame;
    video.write(frame);
/*
    cv::warpAffine(frame, rot, rot_mat, cv::Point2d());
*/
/* 90CW */
    cv::transpose(frame,rot);
    cv::flip(rot,rot,1);

/* 90CCW 
    cv::transpose(frame,rot);
    cv::flip(rot,rot,0);
*/        
    cv::imshow("Cam",rot);

    struct timeval cur;
    gettimeofday(&cur, NULL);

    float dt = (float)(cur.tv_sec-tm.tv_sec) + (float)(cur.tv_usec - tm.tv_usec)/1000000.0;
    float fps = 1.0/dt;
    tm = cur;

    printf("frame: %6ld, tm:%ld.%06ld dt:%5f fps: %0.1f\n", i, cur.tv_sec, cur.tv_usec, dt*1000.0, fps); 


    char k = (char)cv::waitKey(1);
    if (k==27) break;
    
  }  
  return 0;  
}
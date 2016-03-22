#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
#include<sys/time.h>
#include<vector>
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/warpers.hpp"
#include <pthread.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

                     
using namespace std;
using namespace cv;


static void *compass_main(void *p);
volatile int compass_value;

 
int main(int argc, char *argv[]) {

  if (argc==1) {
    printf("Need pano image file name\n");
    return -1;
  }  

/*
  cv::VideoCapture cap(argv[1]);
  if ( !cap.isOpened() )  // if not success, exit program
    {
       printf("Cannot open the video file\n");
       return -1;
    }
*/

  const int frame_width = 1280;
  const int frame_height = 720;

  cv::namedWindow("Pano", CV_WINDOW_NORMAL);
  cv::resizeWindow("Pano", frame_width,frame_height);
  cv::moveWindow("Pano",0,-20);

  Mat pano = imread(argv[1]);
  if (!pano.data) {
    printf("Error loading pano image from: %s\n", argv[1]);
    return -1;
  }
  
  double scale = (double)frame_height/pano.rows;
  int src_width = round((double)frame_width/scale);
  printf("Pano: %d x %d, downscale: %0.2lf, sec_width=%d\n", pano.cols, pano.rows, scale,src_width);

  //init thread
  pthread_attr_t threadAttr;
  pthread_attr_init(&threadAttr);
  pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_DETACHED); 
  
  pthread_t compass_thread;

  int pres = pthread_create(&compass_thread, &threadAttr, compass_main, (void *)0);
  if(pres!=0) {
    printf("Could not create compass thread\n");
    return -1;
  }

  int x = 0;
  int basex = 0;
  int prev_x = -1;
  cv::Mat res;

  while (1) {
  
    if (prev_x != x) {

      while (x<0) x+=pano.cols;
      
      if (x+src_width < pano.cols+1) {
        //ok, just resize
        cv::Rect myRect(x,0,src_width, pano.rows);
        resize(pano(myRect), res, Size(frame_width, frame_height));

      } else {
        //wraps around
        cv::Mat mx( pano.rows, src_width, CV_8UC3, Scalar(0));
        cv::Rect rectA(x,0,pano.cols-x,pano.rows);
        cv::Rect tgtA(0,0,pano.cols-x,pano.rows);

        pano(rectA).copyTo(mx(tgtA));

        cv::Rect rectB(0,0,src_width - (pano.cols-x),pano.rows);
        cv::Rect tgtB(pano.cols-x,0,src_width - (pano.cols-x),pano.rows);
        
        pano(rectB).copyTo(mx(tgtB));

        resize(mx, res, Size(frame_width, frame_height));
      }      


/*
      detail::CylindricalWarper w(700);
      cv::Mat dst;

      Mat K = (Mat_<float>(3,3) << 700,0,(float)frame_width/2.0, 0,700,(float)frame_height/2.0, 0,0,1);
      Mat R = (Mat_<float>(3,3) << 1,0,0,       0,1,0,       0,0,1.0);

      w.warp(res, K, R, INTER_NEAREST, BORDER_CONSTANT, dst);

      imshow("Pano", dst);        
*/
      imshow("Pano", res);        

      prev_x = x;
    }
//      imshow("Pano", res);
    
    float angle = (float)compass_value/10;
    x = (float)pano.cols*angle/360+basex;

    printf("\t\t\t\t\t\t\tx: %d, angle: %0.2f,basex=%d, prevx=%d\r",x,angle,basex,prev_x);    

    char k = (char)cv::waitKey(10);

    if (k==27) break;
    if (k=='q') basex-=50;
    if (k=='w') basex+=50;

  }
  return 0;
  
}

static void *compass_main(void *p) {
  printf("Compass thread start\n");
  
  int fd = open("/dev/ttyUSB0", O_RDWR);
  if (!fd) {
    printf("compass: could not open device\n");
  }     
  
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("compass: error %d from tcgetattr", errno);
                return NULL;
        }

	const int speed = 38400;


        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
//        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("compass: error %d from tcsetattr", errno);
                return NULL;
        }

  char line[1024];
  char *ln;
  ln = &(line[0]);

  FILE *f = fdopen(fd, "r");

  unsigned long _lyaw=-1;
  unsigned long num = 30;

  double headDegrees = -1;
  long   gyro_z = 0;

  long gx,gy,gz;
  int once = 1;


  while (1) {
    size_t n;
    size_t c = getline(&ln, &n, f);
    if (c) {
      char *p;
      if (p = strstr(ln, "compass: cal: ")) {

        long cx, cy, cz;
        if (3!=sscanf(p+14, "%ld %ld %ld", &cx, &cy, &cz)) {
          continue; //ignore
        }

        double heading = atan2(cy,cx);
        if (heading<0) 		heading+=2*M_PI;
        if (heading>2*M_PI)	heading-=2*M_PI;

        headDegrees = heading * 180.0/M_PI;

        //printf("compass: %0.1lf (%ld %ld %ld)\n",headDegrees, cx,cy,cz);

      } else if(p=strstr(ln, "gyro: raw:")) {
        if(3!=sscanf(p+11, "%ld %ld %ld", &gx, &gy, &gz)) {
          continue;
        }
        //printf("gyro: %ld %ld %ld\n",gx,gy,gz);        
        gyro_z = gz;
      } else {
      //  printf("%s",ln);
      }

      if (abs(gyro_z)>200) {
        //moving
        printf("move : %0.1lf gyro=%ld\n", headDegrees, gyro_z);
        compass_value = round(headDegrees*10.0);        
      } else {
        //still or slowly
        printf("stand: %0.1lf gyro=%ld last_compass=%0.1f\n",headDegrees, gyro_z, (float)compass_value/10.0);
        //init or move
        if(once && headDegrees!=-1.0) {
          once = 0;
          compass_value = round(headDegrees*10.0);        
        }
      }     
    }
  }
  
}
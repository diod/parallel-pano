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


const int frame_width = 1280;
const int frame_height = 720;


static void *compass_main(void *p);
volatile int compass_value;

 
int main(int argc, char *argv[]) {

  if (argc<3) {
    printf("Usage: %s <pano.png> <video.avi>\n\n", argv[0]);
    printf("Need pano image file name\n");
    return -1;
  }  

  /* read pano */
  Mat pano = imread(argv[1]);
  if (!pano.data) {
    printf("Error loading pano image from: %s\n", argv[1]);
    return -1;
  }
  double scale = (double)frame_height/pano.rows;
  int src_width = round((double)frame_width/scale);
  printf("Pano: %d x %d, downscale: %0.2lf, sec_width=%d\n", pano.cols, pano.rows, scale,src_width);

  /* open video */
  cv::VideoCapture cap(argv[2]);
  if ( !cap.isOpened() )  // if not success, exit program
    {
       printf("Cannot open the video file: %s\n",argv[2]);
       return -1;
    }


  cv::namedWindow("Pano", CV_WINDOW_FULLSCREEN);
  cv::resizeWindow("Pano", 1920,1080);
  waitKey(1);
  cv::moveWindow("Pano",-1,-5);
  waitKey(1);

  

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

  //prepare blend mask;
  int dcols = 368+8;
  int drows = 505+16;

  Mat blend(drows,dcols,CV_8U,Scalar::all(0));
  for(int bly=10;bly<drows;bly++) {
    for(int blx=8;blx<dcols-8;blx++) {
      char d=255;
      int edge_dist = min(blx+5,bly-10);
      edge_dist = min(edge_dist, drows-bly-22);
      edge_dist = min(edge_dist, dcols-blx-8);

      if (edge_dist < 15) {
        d = saturate_cast<uchar>( 255.0/15.0*(float)edge_dist);
      }
      
      blend.at<uchar>(bly,blx) = d;
    }
  } 

//  cv::namedWindow("Blend", CV_WINDOW_NORMAL);
//  imshow("Blend", blend);
//  waitKey(1);

  int x = 0;
  double baseang = 82;
  int prev_x = -1;
  cv::Mat res;

  while (1) {
    float angle = (float)compass_value/10+baseang;
    if (angle<0)	angle+=360;
    if (angle>=360) 	angle-=360;
  
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
      prev_x = x;
    }      

    printf("init_stage\n");
    Mat stage(res.rows, res.cols, CV_8UC3);
    res.copyTo(stage);
    

    //video
    printf("read_video_frame\n");
    Mat frame;
    cap.read(frame);
    if (!cap.read(frame)) {
        cap.set(CV_CAP_PROP_POS_FRAMES,10);
        if (!cap.read(frame)) {
          printf("video: could not read frame\n");
        } 
    }

    if (frame.data && (abs(angle-128.0) < 1.2)) {
        printf("in_video\n");
        detail::CylindricalWarper w(700*0.815);

        int dcols = 368+8;
        int drows = 505+16;

        int dl = 8;
        int dt = 10;
        
        int vx = 2270 + (frame_width-dcols)/2; //in global pano
        int vy = (frame_height-drows)/2+3;
        
        int locx = (vx-x-frame_width/2)*scale+frame_width/2;
        
        //map to screen
        if ((locx > 0) && (locx < frame_width)) {


          double alpha=1.42;
          char beta = 6;
/*
          for(int imy=0; imy<frame.rows; imy++) {
            for(int imx=0;imx<frame.cols; imx++) {
              for(int imc=0; imc<3; imc++) {
                frame.at<Vec3b>(imy,imx)[imc] =
                  saturate_cast<uchar>( alpha*( frame.at<Vec3b>(imy,imx)[imc] ) -beta ); }
            }
          }
*/

          cv::Mat dst;
  
//          float centerx = (float)frame.cols/2.0 - (vx-x-frame_width/2)/0.815; //if in center
          float centerx = frame.cols/2.0; //-(locx-frame_width/2)/5.0;
          
          printf("x: %d vx:%d centerx: %0.3f\n",x,vx,centerx);
          Mat K = (Mat_<float>(3,3) << 700,0, centerx, 0,700,(float)frame.rows/2.0-150, 0,0,1);
          Mat R = (Mat_<float>(3,3) << 1,0,0,       0,1,0,       0,0,1.0);

          w.warp(frame, K, R, INTER_NEAREST, BORDER_CONSTANT, dst);
          
          dcols = dst.cols;
          drows = dst.rows;
          int dw = dcols-dl;
          int dh = drows-2*dt;

          Rect stgt( dl, dt, dw, dh);      
          
          printf("dst: %d x %d\n", dst.cols, dst.rows);

          Rect vtgt( locx, vy, dw, dh);
          printf("video rect: %d %d %d %d\n\n", (frame_width-dst.cols)/2, (frame_height-dst.rows)/2+1, dw, dh);
/*
//          dst(stgt).copyTo(res(vtgt));
*/
          
          for(int imy=0;imy<dh;imy++) {
            for(int imx=0;imx<dw;imx++) {
              int tx = imx + locx;
              int ty = imy + vy;
              int sx = imx + dl;
              int sy = imy + dt;
              
              for(int imc=0;imc<3;imc++) {
                uchar _tmp = saturate_cast<uchar>( alpha*( dst.at<Vec3b>(sy,sx)[imc] -beta )); 
                
                stage.at<Vec3b>(ty,tx)[imc] = ((int)blend.at<uchar>(imy,imx)*(int)_tmp + (int)(255-blend.at<uchar>(imy,imx)) * res.at<Vec3b>(ty,tx)[imc])/255.0;
              }
            }
          }
          
          

        }
      }

      

/*
      detail::CylindricalWarper w(700);
      cv::Mat dst;

      Mat K = (Mat_<float>(3,3) << 700,0,(float)frame_width/2.0, 0,700,(float)frame_height/2.0, 0,0,1);
      Mat R = (Mat_<float>(3,3) << 1,0,0,       0,1,0,       0,0,1.0);

      w.warp(res, K, R, INTER_NEAREST, BORDER_CONSTANT, dst);

      imshow("Pano", dst);        
*/

    Mat output;
    cv::Rect myRect(x,0,frame_width, frame_height);
    resize(stage, output, Size(1920, 1080));


    imshow("Pano", output);        
//    cv::resizeWindow("Pano", 1920,1080);
//    cv::moveWindow("Pano",-1,-5);

//      imshow("Pano", res);
    
    x = (float)pano.cols*(angle)/360;

    printf("\t\t\t\t\t\t\tx: %d, angle: %0.2f baseang: %6.1f, prevx: %d\r",x,angle,baseang,prev_x);    

    char k = (char)cv::waitKey(10);

    if (k==27) break;
    if (k=='q') baseang-=2.0;
    if (k=='w') baseang+=2.0;

  }
  return 0;
  
}

static void *compass_main(void *p) {
  printf("Compass thread start\n");
  
  int fd = open("/dev/ttyUSB1", O_RDWR);
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

        //unbias
        //cx 2550+4100 = 2740
        cx-=3335;
        //cy -2780 -1000 = 
        cy+=2205;


        double heading = atan2((float)cy/1.11,(float)cx/1.57);
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
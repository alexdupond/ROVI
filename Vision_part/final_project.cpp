#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <cstdio>
#include <ctime>

#include <iostream>

using namespace cv;
using namespace std;

RNG rng(12345);
void help(char** argv)
{
    cout << "\nThis program gets you started reading a sequence of images using cv::VideoCapture.\n"
         << "Image sequences are a common way to distribute video data sets for computer vision.\n"
         << "Usage: " << argv[0] << " <path to the first image in the sequence>\n"
         << "example: " << argv[0] << " marker_color_%02.png\n"
         << "q,Q,esc -- quit\n"
         << "\tThis is a starter sample, to get you up and going in a copy paste fashion\n"
         << endl;
}

int main(int argc, char** argv)
{
    if(argc != 2)
    {
      help(argv);
      return 1;
    }

    string arg = argv[1];
    VideoCapture sequence(arg);
    if (!sequence.isOpened())
    {
      cerr << "Failed to open Image Sequence!\n" << endl;
      return 1;
    }

    Mat image;
    namedWindow("Image | q or esc to quit", CV_WINDOW_NORMAL);

    double avg_time = 0;
    double count = 0;

    for(;;)
    {
      sequence >> image;
      if(image.empty())
      {
          cout << "End of Sequence" << endl;
          break;
      }

      std::clock_t start;
      double duration = 0;

      start = std::clock();

      Mat hsv, blured_img;
      Mat detected_egdes, dst, src, gray;
      dst.create( src.size(), src.type() );
      src = image;
    //  imshow("Original", image);

//      imshow("Original", image);
      cvtColor(image, hsv, CV_BGR2HSV);
      cvtColor(image, gray, CV_BGR2GRAY);

      //imshow("Gray", gray);

      vector<Mat> hsv_planes;
      split(hsv, hsv_planes);
      Mat h = hsv_planes[0];
      Mat s = hsv_planes[1];
      Mat v = hsv_planes[2];
      imshow("h", h);

      int lowThreshold = 10;
      int ratio = 3;
      int kernal_size = 3;

      // bluring the image
      medianBlur(h, blured_img, 13);
      imshow("Blurred", blured_img);

      Canny(blured_img, detected_egdes, lowThreshold, lowThreshold*ratio, kernal_size);
      dst = Scalar::all(0);
      src.copyTo(dst, detected_egdes);
      imshow("Edges", dst);
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      findContours( detected_egdes, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );




      vector<Point> points;
      Point redpoint(0,0);
      Mat drawing = Mat::zeros( detected_egdes.size(), CV_8UC3 );
      for( int i = 0; i < (int)contours.size(); i++ )
          {
            double area = contourArea(contours[i]);
            double perimeter = arcLength(contours[i], false);
            double circle_detect = (4*CV_PI*area)/(pow(perimeter, 2));

            if(circle_detect > 0.8 && (area > 2300)){
              Moments m = moments(contours[i], true);
              int cx = m.m10/m.m00;
              int cy = m.m01/m.m00;
              Point center(cx,cy);
              points.push_back(center);
              Scalar color = image.at<Vec3b>(cy, cx);
              if(color[2] > 80){
                redpoint = center;
                circle( drawing, center, 3, color, -1, 8, 0 );
              }else{
                points.push_back(center);
                drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
                circle( drawing, center, 3, color, -1, 8, 0 );
              }
          }
          }
      Point p3(0,0);
      double dist = 0;
      for (int i = 0; i < (int)points.size(); i++) {
        if(sqrt(pow(redpoint.x-points[i].x,2)+pow(redpoint.y-points[i].y, 2)) > dist){
          dist = sqrt(pow(redpoint.x-points[i].x,2)+pow(redpoint.y-points[i].y, 2));
          p3.x = points[i].x;
          p3.y = points[i].y;
        }
      }

      Scalar newcolor = Scalar(255, 0,0);
      circle( drawing, p3, 3, newcolor, -1, 8, 0 );
      Point p2(0,0);
      Point p4(0,0);

      for (size_t i = 0; i < points.size(); i++) {
    //    if(points[i].x != p3.x && points[i].y != p3.y){
          if((redpoint.x - p3.x) < 0 ){
            if((redpoint.y - p3.y) < 0){
              if(points[i].x > redpoint.x && points[i].y < p3.y)
                p2 = points[i];
              else if(points[i].y > redpoint.y && points[i].x < p3.x)
                p4 = points[i];
            }else{
              if(points[i].y < redpoint.y && points[i].x < p3.x)
                p2 = points[i];
              else if(points[i].x > redpoint.x && points[i].y > p3.y)
                p4 = points[i];
            }
          }else{
            if((redpoint.y - p3.y) < 0){
              if(points[i].x > p3.x && points[i].y > redpoint.y)
                p2 = points[i];
              else if(points[i].y < p3.y && points[i].x < redpoint.x)
                p4 = points[i];
            }else{
              if(points[i].x < redpoint.x && points[i].y > p3.y)
                p2 = points[i];
              else if(points[i].y < redpoint.y && points[i].x > p3.x)
                p4 = points[i];
            }
          }
    //    }
      }



      newcolor = Scalar(0, 0,255);
      circle( drawing, p2, 3, newcolor, -1, 8, 0 );
      newcolor = Scalar(0, 255,0);
      circle( drawing, p4, 3, newcolor, -1, 8, 0 );


       // Show in a window
       namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
       imshow( "Contours", drawing );

       duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

       avg_time +=duration;
       count++;

        char key = (char)waitKey(500);
        if(key == 'q' || key == 'Q' || key == 27)
            break;
      }

      cout << avg_time/count << endl;

    return 0;
}

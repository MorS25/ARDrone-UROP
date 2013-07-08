#include <stdio.h>
#include "video_processing.h"

int init = 0;
int height = 0;
int width = 0;
int lastX = -1;
int lastY = -1;
IplImage* trackingLine;
IplImage* info = NULL;
CvFont font;

int convert_x(int x)
{
  return x - width/2;
}

int convert_y(int y)
{
  return height/2 - y;
}

IplImage* getThresholdedImage(IplImage* in)
{
  IplImage* out = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
  cvInRangeS(in, cvScalar(0,123,119,0), cvScalar(4,205,256,0), out);
  //cvInRangeS(in, cvScalar(144,122,125,0), cvScalar(167,171,203,0), out);
  return out;
}

void trackObject(IplImage* in)
{
  CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
  cvMoments(in, moments, 1);
  double moment10 = cvGetSpatialMoment(moments, 1, 0);
  double moment01 = cvGetSpatialMoment(moments, 0, 1);
  double area = cvGetCentralMoment(moments, 0, 0);

  if(area > 1000)
  {
    int posX = moment10/area;
    int posY = moment01/area;

    if(lastX >=0 && lastY >= 0 && posX >= 0 && posY >= 0)
    {
      char *str = malloc(50*sizeof(char));
      sprintf(str, "X: %dpx, Y: %dpx", convert_x(posX), convert_y(posY));
      cvLine(trackingLine, cvPoint(posX,posY), cvPoint(lastX, lastY),
             cvScalar(0,0,255,0),4,8,0);
      cvPutText(info, str, cvPoint(15,15), 
                           &font, cvScalar(200,200,250,0));
      free(str);
    }

    lastX = posX;
    lastY = posY;
  }

  free(moments);
}

IplImage* process(IplImage* in)
{
  if(init == 0)
  {
    height = in->height;
    width = in->width;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7, 0, 1, 8);
    init = 1;
  }

  cvSmooth(in, in, CV_GAUSSIAN, 3, 3, 0, 0);
  IplImage* hsv = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 3);
  cvCvtColor(in, hsv, CV_BGR2HSV);
  IplImage *out = getThresholdedImage(hsv);
  cvSmooth(out, out, CV_GAUSSIAN, 3, 3, 0, 0);

  trackingLine = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 3);
  info = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 3);

  trackObject(out);

  cvLine(info, cvPoint(0,in->height/2), cvPoint(in->width, in->height/2),
         cvScalar(255,0,0,0),1,8,0);
  cvLine(info, cvPoint(in->width/2,0), cvPoint(in->width/2, in->height),
         cvScalar(255,0,0,0),1,8,0);
  cvAdd(in, trackingLine, in, NULL);
  cvAdd(in, info, in, NULL);

  return out;
}

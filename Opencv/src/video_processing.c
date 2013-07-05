#include "video_processing.h"

IplImage* trackingLine;
int lastX = -1;
int lastY = -1;

IplImage* getThresholdedImage(IplImage* in)
{
  IplImage* out = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
  cvInRangeS(in, cvScalar(0,123,119,0), cvScalar(4,205,256,0), out);
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
      cvLine(trackingLine, cvPoint(posX,posY), cvPoint(lastX, lastY),
             cvScalar(0,0,255,0),4,8,0);
    }

    lastX = posX;
    lastY = posY;
  }

  free(moments);
}

IplImage* process(IplImage* in)
{
  cvSmooth(in, in, CV_GAUSSIAN, 3, 3, 0, 0);
  IplImage* hsv = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 3);
  cvCvtColor(in, hsv, CV_BGR2HSV);
  IplImage *out = getThresholdedImage(hsv);
  cvSmooth(out, out, CV_GAUSSIAN, 3, 3, 0, 0);
  trackingLine = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 3);
  trackObject(out);
  cvAdd(in, trackingLine, in, NULL);
  return out;
}

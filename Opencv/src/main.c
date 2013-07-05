#include <stdio.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include "video_processing.h"

int main(int argc, char** argv)
{
  CvCapture *capture = 0;
  capture = cvCaptureFromCAM(0);

  cvNamedWindow("Video", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Ball", CV_WINDOW_AUTOSIZE);

  IplImage *in = 0;
  IplImage *out;

  while(1)
  {
    in = cvQueryFrame(capture);
    in = cvCloneImage(in);

    out = process(in);

    cvShowImage("Video", in);
    cvShowImage("Ball", out);
    cvReleaseImage(&in);
    cvReleaseImage(&out);
    int c = cvWaitKey(10);
    if((char)c==27) break;
  }

  cvDestroyAllWindows();
  cvReleaseCapture(&capture);
  return 0;
}

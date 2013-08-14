#include <stdio.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include "video_processing3.h"

int main(int argc, char** argv)
{

  CvCapture *capture = 0;
  capture = cvCaptureFromCAM(0);

  cvNamedWindow("Video", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Detect", CV_WINDOW_AUTOSIZE);

  IplImage *in = 0;
  IplImage *out;
  
  int frame = 0;

  while(1)
  {
    in = cvQueryFrame(capture);
    in = cvCloneImage(in);

    if(frame < 2)
    {
      cvReleaseImage(&in);
      frame++;
      continue;
    }
    if(frame == 2)
    {
      if(init(in) != 0)
      {
        cvReleaseImage(&in);
        break;      
      }
      frame++;
    }
    else
    {
      out = process(in);

      cvShowImage("Video", in);
      cvShowImage("Detect", out);
      cvReleaseImage(&out);
    }

    cvReleaseImage(&in);
    int c = cvWaitKey(10);
    if((char)c==27) break;
  }

  cvDestroyAllWindows();
  cvReleaseCapture(&capture);
  return 0;
}

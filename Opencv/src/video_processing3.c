#include <stdio.h>
#include "video_processing3.h"

static int height = 0;
static int width = 0;
static CvFont font;
static CvHistogram* colorhist;
static CvRect roi;
static uint8_t drawing = 0;

int convert_x(int x)
{
  return x - width/2;
}

int convert_y(int y)
{
  return height/2 - y;
}

void draw_box(CvArr* img, CvRect rect, CvScalar color)
{
  cvRectangle(img, cvPoint(rect.x, rect.y), 
                   cvPoint(rect.x + rect.width, rect.y + rect.height), 
                   color, 1, 8, 0);
}

void mouse_callback(int event, int x, int y, int flags, void* param)
{
  switch(event)
  {
    case CV_EVENT_MOUSEMOVE:
      if(drawing)
      {
        roi.width = x - roi.x;
        roi.height = y - roi.y;
      }
    break;

    case CV_EVENT_LBUTTONDOWN:
      drawing = 1;
      roi = cvRect(x, y, 0, 0);
    break;

    case CV_EVENT_LBUTTONUP:
      drawing = 0;
      if(roi.width < 0)
      {
        roi.x += roi.width;
        roi.width *= -1;
      }
      if(roi.height < 0)
      {
        roi.y += roi.height;
        roi.height *= -1;
      }
    break;
  }
}

IplImage* getChannel(IplImage* img, int c)
{
  int i = 0;
  IplImage* channels[4];
  
  for (i = 0; i < 4; i++)
  { 
    if(i < img->nChannels)
      channels[i] = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
    else
      channels[i] = NULL;
  }

  cvSplit(img, channels[0], channels[1], channels[2], channels[3]);
  
  for (i = 0; i < 4; i++)
  {
    if(i != c && channels[i] != NULL)
      cvReleaseImage(&channels[i]);
  } 

  return channels[c];
}

CvHistogram* getHueHistogram(CvArr *img, int minSat)
{
  IplImage *mask, *h, *s;
  h = s = mask = NULL;
  IplImage *hsv = cvCreateImage(cvGetSize((IplImage*)img),IPL_DEPTH_8U,3);

  int histSizes[] = {256};
  float hranges[] = {0.0, 180.0};
  float *ranges[] = {hranges};
  
  CvHistogram *hist = cvCreateHist(1,histSizes,CV_HIST_ARRAY,ranges,1);

  cvCvtColor((IplImage*)img, hsv, CV_BGR2HSV);

  h = getChannel(hsv, 0);
  
  if(minSat > 0)
  {
    mask = cvCreateImage(cvGetSize(hsv),IPL_DEPTH_8U,1);
    s = getChannel(hsv, 1);
    cvThreshold(s, mask, minSat, 255, CV_THRESH_BINARY);
  }

  cvCalcHist(&h, hist, 0, mask);

  cvReleaseImage(&h);
  cvReleaseImage(&hsv);

  if(s != NULL)
    cvReleaseImage(&s);
  if(mask != NULL)
    cvReleaseImage(&mask);

  return hist;
}

int init(CvArr* in)
{
  IplImage *temp = cvCloneImage(in);
  height = ((IplImage*)in)->height;
  width = ((IplImage*)in)->width;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7, 0, 1, 8);
  cvNamedWindow("Initialisation", CV_WINDOW_AUTOSIZE);
  cvShowImage("Initialisation", (IplImage*)temp);
  cvSetMouseCallback("Initialisation", mouse_callback, (void*)temp);

  while(1)
  {
    if(drawing)
    {
      temp = cvCloneImage(in);
      draw_box(temp, roi, cvScalar(0, 255, 0, 0));
      cvShowImage("Initialisation", temp);
    }
    char c;
    c = cvWaitKey(15);
    if(c == 27) return 1; 
    if(c == 10) break;
  }
  cvDestroyWindow("Initialisation");
  cvReleaseImage(&temp);

  cvSetImageROI(in, roi);
  colorhist = getHueHistogram(in, 65);
  return 0;
}

CvArr* process(CvArr* in)
{
  char *str;
  IplImage *input = (IplImage*)in;
  IplImage *hsv = cvCreateImage(cvGetSize((IplImage*)in),IPL_DEPTH_8U,3);
  IplImage *result = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);

  cvCvtColor(input, hsv, CV_BGR2HSV);
  IplImage *s = getChannel(hsv, 1);
  IplImage *h = getChannel(hsv, 0);


  cvThreshold(s, s, 65, 255, CV_THRESH_BINARY);


  cvCalcBackProject(&h, result, colorhist);
  cvAnd(result, s, result, NULL);

  CvConnectedComp comp;
  CvBox2D box;
  CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.5);
  cvCamShift(result, roi, criteria, &comp, &box);

  cvLine(input, cvPoint(0,height/2), cvPoint(width, height/2),
         cvScalar(255,0,0,0),1,8,0);
  cvLine(input, cvPoint(width/2,0), cvPoint(width/2, height),
         cvScalar(255,0,0,0),1,8,0);
  draw_box(input, comp.rect, cvScalar(0, 255, 0, 0));
  cvEllipseBox(input, box, cvScalar(0,0,0,0), 1, 8, 0);
  cvCircle(input, cvPointFrom32f(box.center), 1, cvScalar(128,128,128,0), 1, 8, 0);

  str = malloc(500*sizeof(char));
  sprintf(str, "X: %dpx, Y: %dpx Width: %f, Height: %f, Angle: %f", 
    convert_x(box.center.x), convert_y(box.center.y), box.size.width, box.size.height, box.angle);

  cvPutText(input, str, cvPoint(15,15), &font, cvScalar(0,0,0,0));
  free(str);

  if(comp.rect.width > 0 && comp.rect.height > 0)
    roi = comp.rect;

  cvReleaseImage(&h);
  cvReleaseImage(&s);
  cvReleaseImage(&hsv);

  return result;
}

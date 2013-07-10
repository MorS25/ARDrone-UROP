#include <stdio.h>
#include <math.h>
#include "video_processing2.h"

int init = 0;
IplImage* prev = NULL;
IplImage* features;
IplImage* info = NULL;
CvMemStorage *storage;
CvFont font;

/*int convert_x(int x)
{
  return x - width/2;
}

int convert_y(int y)
{
  return height/2 - y;
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
}*/
IplImage* extractSURF(IplImage *in)
{
  storage = cvCreateMemStorage(0);
  
  int i = 0;
  IplImage *out = cvCloneImage(in);
  CvMat *gray = cvCreateMat(in->height, in->width, CV_8UC1);
  cvCvtColor(in, gray, CV_BGR2GRAY);
  CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
 
  CvSURFParams params = cvSURFParams(500, 1); 
  cvExtractSURF(gray, 0, &imageKeypoints, &imageDescriptors, storage, params, 0);
  
  for( i = 0; i < imageKeypoints->total; i++ )
  {
    CvSURFPoint *r = (CvSURFPoint*)cvGetSeqElem(imageKeypoints, i);
    CvPoint center;
    int radius;
    CvScalar red = cvScalar(0,0,255,0);
    center.x = cvRound(r->pt.x);
    center.y = cvRound(r->pt.y);
    radius = cvRound(r->size*1.2/9.*2);
    cvCircle(out, center, radius, red, 1, 8, 0);
  }

  cvReleaseMemStorage(&storage);
  return out;
}

IplImage* process(IplImage* in)
{
  if(init == 0)
  {
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7, 0, 1, 8);
    init = 1;
  }
  //IplImage* out = extractSURF(in);
  IplImage* out = cvCloneImage(in);
  if(prev != NULL)
  {
    IplImage *frame1 = cvCreateImage(cvGetSize(prev), IPL_DEPTH_8U, 1);
    IplImage *eig_img = cvCreateImage(cvGetSize(in), IPL_DEPTH_32F, 1);
    IplImage *temp_img = cvCreateImage(cvGetSize(in), IPL_DEPTH_32F, 1);
    cvCvtColor(prev, frame1, CV_BGR2GRAY);
 
    IplImage *frame2 = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
    cvCvtColor(in, frame2, CV_BGR2GRAY);

    int nbFeatures = 500;
    CvPoint2D32f frame1_features[500];
    CvPoint2D32f frame2_features[500];

    cvGoodFeaturesToTrack(frame1, eig_img, temp_img, frame1_features, 
                          &nbFeatures, 0.01, 0.01, NULL, 3, 0, 0.04);
    
    char optical_flow_found_features[500];
    float optical_flow_feature_errors[500];

    CvTermCriteria optical_flow_termination_criteria 
          = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
    
    IplImage *pyr1 = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
    IplImage *pyr2 = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);

    cvCalcOpticalFlowPyrLK(frame1, frame2, pyr1, pyr2, frame1_features,
                           frame2_features, nbFeatures, cvSize(3,3), 5,
                           optical_flow_found_features, 
                           optical_flow_feature_errors,
                           optical_flow_termination_criteria, 0);

    int i = 0;
    for (i = 0; i < nbFeatures; i++)
    {
      if(optical_flow_found_features[i] == 0) continue;
      CvPoint p,q;
      
      p.x = (int) frame1_features[i].x;
      p.y = (int) frame1_features[i].y;
      q.x = (int) frame2_features[i].x;
      q.y = (int) frame2_features[i].y;
  
      double angle = atan2((double) p.y - q.y, (double) p.x - q.y);
      double hypo = sqrt(pow((p.y - q.y),2) + pow((p.x - q.y),2));
      /*q.x = (int) (p.x - 3 * hypo * cos(angle));
      q.y = (int) (p.y - 3 * hypo * sin(angle));*/
     
      cvLine(out, p, q, cvScalar(0,0,255,0), 1, 8, 0);
    }     

    cvReleaseImage(&prev);
  }  

  prev = cvCloneImage(in);
  return out;
}

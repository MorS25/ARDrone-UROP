#include <stdio.h>
#include <math.h>
#include "video_processing2.h"

IplImage* prev = NULL;

IplImage* process(IplImage* in)
{
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

		CvPoint2D32f frame2_points_to_find[500];
		int frame2_points_to_find_back_index[500];

    int i = 0;
    int j = 0;
    for (i = 0; i < nbFeatures; i++)
    {
      if(optical_flow_found_features[i] 
				&& optical_flow_feature_errors[i]  < 12.0)
			{
				frame2_points_to_find[j] = frame2_features[i];
				frame2_points_to_find_back_index[j] = i;
			}
			else
			{
				optical_flow_found_features[i] = 0;
			}
    }     

    cvReleaseImage(&prev);
  }  

  prev = cvCloneImage(in);
  return out;
}


/*CvPoint p,q;
				
p.x = (int) frame1_features[i].x;
p.y = (int) frame1_features[i].y;
q.x = (int) frame2_features[i].x;
q.y = (int) frame2_features[i].y;

double angle = atan2((double) p.y - q.y, (double) p.x - q.y);
double hypo = sqrt(pow((p.y - q.y),2) + pow((p.x - q.y),2));
q.x = (int) (p.x - 3 * hypo * cos(angle));
q.y = (int) (p.y - 3 * hypo * sin(angle));
  
cvLine(out, p, q, cvScalar(0,0,255,0), 1, 8, 0);*/

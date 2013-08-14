#include <iostream>
#include "video_processing.hpp"

vector<CloudPoint> pointcloud;

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.showCloud(cloud, "cloud");
	viewer.setBackgroundColor(0, 0, 0);
 	//viewer.addCoordinateSystem (1.0);
 	viewer.initCameraParameters ();

  Mat in;
  VideoCapture capture(0);
	namedWindow("Video", CV_WINDOW_AUTOSIZE);

  while(1)
  {
		capture >> in;

		buildCloud(in, pointcloud, cloud, viewer);

		imshow("Video", in);
   	int c = waitKey(10);
   	if((char)c==27) break;
  }


	/*vector<Mat> imgs;

	imgs.push_back(imread("0.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("1.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("2.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("3.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("4.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("5.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("6.jpg", CV_LOAD_IMAGE_COLOR));
	imgs.push_back(imread("7.jpg", CV_LOAD_IMAGE_COLOR));

  for (unsigned int i = 0; i < imgs.size(); i++)
  {
		buildCloud(imgs[i], pointcloud, cloud, viewer);

    int c = waitKey(10);
    if((char)c==27) break;
  }*/

	//runVisualization(pointcloud);
	while(!viewer.wasStopped())
	{
		viewer.spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

  return 0;
}

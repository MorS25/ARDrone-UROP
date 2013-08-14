#ifndef VIDEO_PROCESSING_H
#define VIDEO_PROCESSING_H

#define V3DLIB_ENABLE_SUITESPARSE

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ssba/Math/v3d_linear.h>
#include <ssba/Base/v3d_vrmlio.h>
#include <ssba/Geometry/v3d_metricbundle.h>

using namespace cv;
using namespace std;
using namespace V3D; 

struct CloudPoint {
	Point3d pt;
	set<long> index_of_2d_origin;
	Vec3b color;
};

void buildCloud(Mat&, vector<CloudPoint> &pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::visualization::PCLVisualizer &viewer);
void process(Mat&, Mat&);
void runVisualization(const vector<CloudPoint> &pointcloud);

#endif

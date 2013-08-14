#include <iostream>
#include <set>
#include "video_processing.hpp"

Mat K, Kinv, distcoeff;
Mat first, second;

vector<Mat> old_views;
vector<Matx34d> Pmats;
vector<vector<KeyPoint> > imgpts;

int height, width;

void initCalibration(Mat &img)
{
	FileStorage fs;
	if(fs.open("calibration.yml", FileStorage::READ)) {
		fs["camera_matrix"] >> K;
		fs["distortion_coefficients"] >> distcoeff;
		Kinv = K.inv();
	
		width = img.cols;	
		height = img.rows;	

		cout << width << endl;
		cout << height << endl;
	}
	else
	{
		cout << "Calibration needed" << endl;
		return;
	}
}

void keypointsToPoints(vector<KeyPoint> &kp, vector<Point2f> &p)
{
	p.clear();
	for(unsigned int i = 0; i < kp.size(); i++)
		p.push_back(kp[i].pt);
}

void getKeyPoints(Mat &img, vector<KeyPoint> &keypoints)
{
	FastFeatureDetector ffd;
	ffd.detect(img, keypoints);
}

vector<DMatch> getMatches(Mat &first, Mat &second, vector<KeyPoint> &first_keypoints, vector<KeyPoint> &second_keypoints)
{
	vector<Point2f> first_points;
	vector<DMatch> matches;

	keypointsToPoints(first_keypoints, first_points);

	vector<Point2f> second_points(first_points.size());

	Mat firstgray, secondgray;
	if(second.channels() == 3)
	{
		cvtColor(first, firstgray, CV_RGB2GRAY);
		cvtColor(second, secondgray, CV_RGB2GRAY);
	}
	else
	{
		firstgray = first;
		secondgray = second;
	}

	vector<uchar> vstatus;
	vector<float> verror;

  calcOpticalFlowPyrLK(firstgray, secondgray, 
													first_points, second_points, vstatus, verror);

	vector<Point2f> points_to_find;
	vector<int> points_to_find_back_index;

  for (unsigned int i = 0; i < vstatus.size(); i++)
  {
    if(vstatus[i] && verror[i]  < 12.0)
		{
			points_to_find.push_back(second_points[i]);
			points_to_find_back_index.push_back(i);
		}
		else
		{
			vstatus[i] = 0;
		}
  }

	Mat points_to_find_flat = Mat(points_to_find)
															.reshape(1, points_to_find.size());

	vector<Point2f> second_features;
	keypointsToPoints(second_keypoints, second_features);

	Mat second_features_flat = Mat(second_features)
														.reshape(1, second_features.size());

	BFMatcher matcher(CV_L2 );
	vector< vector<DMatch> > nearest_neighbours;
	matcher.radiusMatch(points_to_find_flat, second_features_flat, 
												nearest_neighbours, 2.0f);

	set<int> found_points;
	for (unsigned int i = 0; i < nearest_neighbours.size(); i++)
	{
		DMatch _m;
		if(nearest_neighbours[i].size() == 1)
			_m = nearest_neighbours[i][0];
		else if(nearest_neighbours[i].size() > 1)
		{
			double ratio = nearest_neighbours[i][0].distance 
										/ nearest_neighbours[i][1].distance;

			if(ratio < 0.7)
				_m = nearest_neighbours[i][0];
			else
				continue;
		}
		else
			continue;

		if(found_points.find(_m.trainIdx) == found_points.end())
		{
			_m.queryIdx = points_to_find_back_index[_m.queryIdx];
			matches.push_back(_m);
			found_points.insert(_m.trainIdx);
		}
	}

	return matches;
}

bool checkCoherentRotation(Mat_<double> &R)
{
	if(fabsf(determinant(R)) - 1.0 > 1e-07)
	{
		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
		return false;
	}

	return true;
}

void findCameraMatrices(const Mat &K, const vector<Point2f> &imgpts1, const vector<Point2f> &imgpts2, Matx34d &P, Matx34d &P1)
{
	Mat F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
	Mat_<double> E = K.t() * F * K; 

	cv::SVD svd(E);
	Matx33d W(0, -1, 0,
						1,  0, 0,
						0,  0, 1);	

	Mat_<double> R = svd.u * Mat(W).inv() * svd.vt;
	//Mat_<double> t = svd.vt.t() * Mat(W) * Mat::diag(svd.w) * svd.vt;
	Mat_<double> t = svd.u.col(2);

	if(!checkCoherentRotation(R))
	{
		cout << "Resulting rotation is not coherent" << endl;
		P1 = 0;
		return;
	}

	P1 = Matx34d(R(0,0), R(0,1), R(0,2), t(0),//t(2,1),
							 R(1,0), R(1,1), R(1,2), t(1),//t(0,2),
						   R(2,0), R(2,1), R(2,2), t(2));//t(1,0));
}

void adjustBundle(vector<CloudPoint> &pointcloud, const Mat &K, vector<Matx34d> &Pmats)
{
	unsigned int size_pmats = Pmats.size();
	unsigned int size_pointcloud = pointcloud.size();

	StdDistortionFunction distortion;

	Matrix3x3d Kmat;	
	makeIdentityMatrix(Kmat);

	Kmat[0][0] = K.at<double>(0,0);
	Kmat[0][1] = K.at<double>(0,1);
	Kmat[0][2] = K.at<double>(0,2);
	Kmat[1][1] = K.at<double>(1,1);
	Kmat[1][2] = K.at<double>(1,2);

	double const f0 = Kmat[0][0];
	Matrix3x3d Knorm = Kmat;
	scaleMatrixIP(1.0/f0, Knorm);
	Knorm[2][2] = 1.0;

	vector<Vector3d> Xs(size_pointcloud);
	for(unsigned int i = 0; i < size_pointcloud; i++)
	{
		Xs[i][0] = pointcloud[i].pt.x;
		Xs[i][1] = pointcloud[i].pt.y;
		Xs[i][2] = pointcloud[i].pt.z;
	}	

	vector<CameraMatrix> cams(size_pmats);
	for(unsigned int i = 0; i < size_pmats; i++)
	{
		Matrix3x3d R;
		Vector3d t;

		Matx34d P = Pmats[i];
		R[0][0] = P(0,0); R[0][1] = P(0,1); R[0][2] = P(0,2); t[0] = P(0,3);
		R[1][0] = P(1,0); R[1][1] = P(1,1); R[1][2] = P(1,2); t[1] = P(1,3);
		R[2][0] = P(2,0); R[2][1] = P(2,1); R[2][2] = P(2,2); t[2] = P(2,3);

		cams[i].setIntrinsic(Knorm);
		cams[i].setRotation(R);
		cams[i].setTranslation(t);
	}

	vector<Vector2d> measurements;
	vector<int> correspondingView;
	vector<int> correspondingPoint;

	for(unsigned int i = 0; i < size_pointcloud; i++)
	{
		for(set<long>::iterator it = pointcloud[i].index_of_2d_origin.begin();
				it != pointcloud[i].index_of_2d_origin.end();
				it++)
		{
			int point = i;
			long idx = *it;

			int view = idx / (height*width); 

			idx -= view *(height*width);

			int y = idx / width;
			int x = idx - (y * width); 

			Vector3d p;	
			p[0] = x;
			p[1] = y;
			p[2] = 1.0;

			scaleVectorIP(1.0/f0, p);
			measurements.push_back(Vector2d(p[0], p[1]));
			correspondingView.push_back(view);
			correspondingPoint.push_back(point);
		}
	}

	double const inlierThreshold = 2.0 / fabs(f0);
	
	Matrix3x3d K0 = cams[0].getIntrinsic();
	
	CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_FOCAL_LENGTH_PP, inlierThreshold, K0, distortion, cams, Xs,
												 measurements, correspondingView, correspondingPoint);

	opt.tau = 1e-3;
	opt.maxIterations = 50;
	opt.minimize();

	cout << "Optimizer status = " << opt.status << endl;

	for(unsigned int i = 0; i < Xs.size(); i++)
	{
		pointcloud[i].pt.x = Xs[i][0];
		pointcloud[i].pt.y = Xs[i][1];
		pointcloud[i].pt.z = Xs[i][2];
	}

	for(unsigned int i = 0; i < size_pmats; i++)
	{
		Matrix3x3d R = cams[i].getRotation();
		Vector3d t = cams[i].getTranslation();

		Matx34d P;

		P(0,0) = R[0][0]; P(0,1) = R[0][1]; P(0,2) = R[0][2]; P(0,3) = t[0];
		P(1,0) = R[1][0]; P(1,1) = R[1][1]; P(1,2) = R[1][2]; P(1,3) = t[1];
		P(2,0) = R[2][0]; P(2,1) = R[2][1]; P(2,2) = R[2][2]; P(2,3) = t[2];

		Pmats[i] = P;
	}
}

Mat_<double> linearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1, double wi, double wi1)
{
	Matx43d A((u.x*P(2,0)-P(0,0))/wi,     (u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,
						(u.y*P(2,0)-P(1,0))/wi,     (u.y*P(2,1)-P(1,1))/wi, 		(u.y*P(2,2)-P(1,2))/wi,
						(u1.x*P1(2,0)-P1(0,0))/wi1, (u1.x*P1(2,1)-P1(0,1))/wi1, (u1.x*P1(2,2)-P1(0,2))/wi1,
						(u1.y*P1(2,0)-P1(1,0))/wi1, (u1.y*P1(2,1)-P1(1,1))/wi1, (u1.y*P1(2,2)-P1(1,2))/wi1);

	Matx41d B(-(u.x*P(2,3)   -P(0,3)/wi),
						-(u.y*P(2,3)   -P(1,3)/wi),
						-(u1.x*P1(2,3) -P1(0,3)/wi1),
						-(u1.y*P1(2,3) -P1(1,3))/wi1);

	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);
	return X;
}

Mat_<double> iterativeLinearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1)
{
	Mat_<double> X;
  Mat_<double> X_ = Mat_<double>(4,1);
	double wi = 1, wi1 = 1;
	for (unsigned int i = 0; i < 10; i++)
	{
		cout << "Ite #" << i << endl;
		X = linearLSTriangulation(u, P, u1, P1, wi, wi1);
		X_(0) = X(0); X_(1) = X(1); X_(2) = X(2); X_(3) = 1.0;
		
		double p2x	= Mat_<double>(Mat_<double>(P).row(2)*X_)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X_)(0);

		cout << "|wi - p2x| = " << fabs(wi - p2x) << endl;
		cout << "|wi1 - p2x1| = " << fabs(wi1 - p2x1) << endl;
		if(fabs(wi - p2x) <= 0.001 && fabs(wi1 - p2x1) <= 0.001) break;

		wi = p2x;
		wi1 = p2x1;
	}

	return X;
}

double triangulatePoints
	( const Mat &img,
		const int idx_img1,
		const int idx_img2,
		const vector<Point2f>& pt_set1,
		const vector<Point2f>& pt_set2, 
		const Mat &K, const Mat &Kinv, 
		const Matx34d &P, const Matx34d &P1,
		vector<CloudPoint> &pointcloud)
{
	bool unfilter = true;
	vector<double> reproj_error;	
	for( unsigned int i = 0; i < pt_set1.size(); i++ )
	{
		Point2f kp = pt_set1[i];
		Point3d u(kp.x, kp.y, 1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u.x = um(0); u.y = um(1); u.z = um(2);

		Point2f kp1 = pt_set2[i];
		Point3d u1(kp1.x, kp1.y, 1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		//Mat_<double> X = iterativeLinearLSTriangulation(u, P, u1, P1);
		Mat_<double> X = linearLSTriangulation(u, P, u1, P1, 1.0, 1.0);
		//cout << "X(" << X(0) << "," << X(1) << "," << X(2) <<  ")" << endl;

    Mat_<double> X_ = Mat_<double>(4,1);
		X_(0) = X(0); X_(1) = X(1); X_(2) = X(2); X_(3) = 1.0;

		Mat_<double> xpt_img = K * Mat(P1) * X_;
		Point2f xpt_img_(xpt_img(0)/xpt_img(2), xpt_img(1)/xpt_img(2));

		double re = norm(xpt_img_ - kp1);
		if(unfilter || re < 30.0)
		{
			reproj_error.push_back(re);

			CloudPoint cp;
			cp.pt = Point3d(X(0), X(1), X(2));
			cp.index_of_2d_origin.insert(idx_img1 * (height*width) + kp.y * width + kp.x);
			cp.index_of_2d_origin.insert(idx_img2 * (height*width) + kp1.y * width + kp1.x);
			//cp.color = img.at<Vec3b>(kp.x, kp.y);
			pointcloud.push_back(cp);
		}
	}

	Scalar me = mean(reproj_error);
	cout << "Return" << endl;
	return me[0];
}

void populatePCLpointcloud(const vector<CloudPoint> &pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::visualization::PCLVisualizer &viewer)
{
	cout << "creating point cloud .." << endl;
	cout << "Nb Points: " << pointcloud.size() << endl;
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(unsigned int i = 0; i < pointcloud.size(); i++)
	{
		//Vec3b rgbv = pointcloud[i].color;
		Vec3b rgbv = Vec3b(255, 255, 255);

		if(pointcloud[i].pt.x != pointcloud[i].pt.x || isnan(pointcloud[i].pt.x) ||
			 pointcloud[i].pt.y != pointcloud[i].pt.y || isnan(pointcloud[i].pt.y) ||
			 pointcloud[i].pt.z != pointcloud[i].pt.z || isnan(pointcloud[i].pt.z) ||
			 fabsf(pointcloud[i].pt.x) > 10.0 ||
			 fabsf(pointcloud[i].pt.y) > 10.0 ||
			 fabsf(pointcloud[i].pt.z) > 10.0) continue;

		pcl::PointXYZRGB pclp;
		pclp.x = pointcloud[i].pt.x;
		pclp.y = pointcloud[i].pt.y;
		pclp.z = pointcloud[i].pt.z;

		uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
		pclp.rgb = *reinterpret_cast<float*>(&rgb);

		cloud->push_back(pclp);
	}

	cloud->width = (uint32_t) cloud->points.size();
	cloud->height = 1;
}

/*void runVisualization(const vector<CloudPoint> &pointcloud)
{
	populatePCLpointcloud(pointcloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud, "cloud");
	while(!viewer.wasStopped()) {}
}*/

void buildCloud(Mat &img, vector<CloudPoint> &pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::visualization::PCLVisualizer &viewer) 
{
	vector<KeyPoint> keypoints;	

	if(old_views.size() == 0)
		initCalibration(img);

	getKeyPoints(img, keypoints);
	imgpts.push_back(keypoints);

	if(old_views.size() > 0)
	{
		Mat prev;
		Matx34d P, P1;
		vector<KeyPoint> prev_keypoints;	
		vector<DMatch> matches;
		vector<vector<DMatch> > all_matches;
	
		for(unsigned int i = 0; i < old_views.size(); i++)
		{
			prev = old_views[i];
			prev_keypoints = imgpts[i];

			matches = getMatches(prev, img, prev_keypoints, keypoints);
			all_matches.push_back(matches);
		}

		vector<Point2f> imgpts1, imgpts2;
		if(old_views.size() == 1)
		{
			P = Matx34d(1.0, 0.0, 0.0, 0.0,
									0.0, 1.0, 0.0, 0.0,
									0.0, 0.0, 1.0, 0.0);

			for( unsigned int i = 0; i < matches.size(); i++ )
			{
				imgpts1.push_back(prev_keypoints[matches[i].queryIdx].pt);
				imgpts2.push_back(keypoints[matches[i].trainIdx].pt);
			}

			findCameraMatrices(K, imgpts1, imgpts2, P, P1);

			double e = triangulatePoints(img, 0, 1, imgpts1, imgpts2, K, Kinv, P, P1, pointcloud);

			cout << "Error = " << e << endl;
			
			Pmats.push_back(P);
			Pmats.push_back(P1);
		}
		else
		{
			vector<Point3f> ppcloud;
			vector<Point2f> imgPoints;
			vector<int> pcloud_status(pointcloud.size(), 0);

			for(unsigned int i = 0; i < old_views.size(); i++)
			{
				vector<DMatch> matches_ = all_matches[i];

				for(unsigned int j = 0; j < matches_.size(); j++)
				{
					int idx_in_old_view = matches_[j].queryIdx;

					long v = i * (height*width) + imgpts[i][idx_in_old_view].pt.y * width + imgpts[i][idx_in_old_view].pt.x;

					for(unsigned int pcldp = 0; pcldp < pointcloud.size(); pcldp++)
					{
						if(pointcloud[pcldp].index_of_2d_origin.find(v) 
								!= pointcloud[pcldp].index_of_2d_origin.end()
							&& pcloud_status[pcldp] == 0)
						{
							ppcloud.push_back(pointcloud[pcldp].pt);

							Point2d pt_ = (imgpts.back())[matches_[j].trainIdx].pt;
							imgPoints.push_back(pt_);

							pcloud_status[pcldp] = 1;
							break;
						}
					}
				}
			}

			cout << "found " << ppcloud.size() << " 3d-2d correspondences" << endl;
			if(ppcloud.size() == 0)
				return;

			Mat_<double> t, rvec, R;
			solvePnPRansac(ppcloud, imgPoints, K, distcoeff, rvec, t, false);
			Rodrigues(rvec, R);

			Pmats.push_back(Matx34d(R(0,0), R(0,1), R(0,2), t(0),
															R(1,0), R(1,1), R(1,2), t(1),
															R(2,0), R(2,1), R(2,2), t(2)));

			// Triangulation with previous views
			for(unsigned int i = 0; i < old_views.size(); i++)
			{
				P  = Pmats[i];
				P1 = Pmats.back();

				prev_keypoints = imgpts[i];
				matches = all_matches[i];

				for( unsigned int j = 0; j < matches.size(); j++ )
				{
					imgpts1.push_back(prev_keypoints[matches[j].queryIdx].pt);
					imgpts2.push_back(keypoints[matches[j].trainIdx].pt);
				}

				double e = triangulatePoints(img, (int)i, (int)old_views.size(), imgpts1, imgpts2, K, Kinv, P, P1, pointcloud);

				cout << "Error = " << e << endl;
			}
		}

		adjustBundle(pointcloud, K, Pmats);
		populatePCLpointcloud(pointcloud, cloud, viewer);
		if(old_views.size() == 1)
		{
			viewer.addPointCloud(cloud, "cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}
		else
			viewer.updatePointCloud(cloud, "cloud");

		viewer.spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	old_views.push_back(img);
}

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "common.h"
#include <io.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;



int main()
{
	cout << "Hello!" << endl;
	//读取目录下所有jpg文件
	FilesIO files(".././img/", ".jpg");
	vector<Mat> imgs = files.getImages();
	//两视图匹配测试
	FeatureMatch matcher(imgs[0], imgs[1]);
	matcher.OFFeatureMatch();//匹配方法
	vector<KeyPoint> l_keypoints(matcher.getKeypoints_L());
	vector<KeyPoint> r_keypoints(matcher.getKeypoints_R());
	vector<DMatch> matches(matcher.getMatches());
	//Mat shows;
	//drawMatches(imgs[0], l_keypoints, imgs[1], r_keypoints, matches, shows);
	//imshow("Matches: ",shows);
	//waitKey(1000);

	vector<Point2d>l_points, r_points;
	for (size_t i = 0; i < matches.size(); i++)
	{
		l_points.push_back(l_keypoints[matches[i].queryIdx].pt);
		r_points.push_back(r_keypoints[matches[i].trainIdx].pt);
	}
	Mat E = findEssentialMat(l_points, r_points, K ,RANSAC, 0.999, 1.0);
	cout << E << endl;
	//3.20日进展： 本质矩阵E 经过SVD分解成R，t, 
	//改 findCameraMatrix函数
	Mat R, t, mask;
	std::vector<cv::Point2d> cam0pnts;
	std::vector<cv::Point2d> cam1pnts;
	// You fill them, both with the same size...
	// You can pick any of the following 2 (your choice)
	cv::Mat pnts3D(1,cam0pnts.size(),CV_64FC4);

	//Notice here a threshold dist is used to filter
	double dist = 50.0;
	recoverPose(E, l_points, r_points, K, R, t, dist,  mask, pnts3D);
	int count = 0;
	vector<MyPoint> pointscloud;

	for (size_t i = 0; i < pnts3D.cols; i++)
	{
		MyPoint temp;
		temp.x = pnts3D.at<Vec4d>(1, i)[0];
		temp.y = pnts3D.at<Vec4d>(1, i)[1];
		temp.z = pnts3D.at<Vec4d>(1, i)[2];
		temp.intensity = 255;
		pointscloud.push_back(temp);
	}
	cout << pointscloud.size() << endl;
	Pt_FileIO write("out.asc",pointscloud,'w');

	
	return 0;
}

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <io.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "base.h"
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
	//drawMatches(imgs[0], l_keypoints, imgs[1], r_keypoints, matches,shows);
	//imshow("Matches: ",shows);
	//waitKey(1000);

	vector<Point2f>l_points, r_points;
	for (size_t i = 0; i < matches.size(); i++)
	{
		l_points.push_back(l_keypoints[matches[i].queryIdx].pt);
		r_points.push_back(r_keypoints[matches[i].trainIdx].pt);
	}
	
	Mat E = findEssentialMat(l_points, r_points, K ,RANSAC, 0.999, 1.0);

	//p101页






	return 0;
}

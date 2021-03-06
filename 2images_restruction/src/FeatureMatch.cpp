
#include "FeatureMatch.h"

void FeatureMatch::OFFeatureMatch()
{
	//Optical flow
	Ptr<FastFeatureDetector> ffd = FastFeatureDetector::create();
	ffd->detect(img1, l_keypoints);
	ffd->detect(img2, r_keypoints);
	vector<Point2f> l_points;//坐标转换
	KeyPoint::convert(l_keypoints, l_points);
	vector<Point2f> r_points(l_points.size());
	//make sure images are grayscale
	Mat prevgray, gray;
	if (img1.channels() == 3) { cvtColor(img1, prevgray, CV_RGB2GRAY); }
	else { prevgray = img1;	gray = img2; }
	//Caculate the optical flow field
	//how each l_points moved across the 2 images
	vector<uchar> vstatus; vector<float> verror;
	calcOpticalFlowPyrLK(prevgray, gray, l_points, r_points, vstatus, verror);
	//First, filter out the points with high error
	vector<Point2f> to_find;
	vector<int> to_find_back_index;
	for (size_t i = 0; i<vstatus.size(); i++) {
		if (vstatus[i] && verror[i] < 12.0) {
			//keep the orignal index of the point in the optical flow array, for future use
			i = int(i);
			to_find_back_index.push_back(i);
			to_find.push_back(r_points[i]);//keep the feature point itself
		}
		else {
			vstatus[i] = 0;//a bad flow
		}
	}
	//for each r_points see which detected feature it belongs to 
	Mat to_find_flat = Mat(to_find).reshape(1, int(to_find.size()));

	vector<Point2f> right_features;
	KeyPoint::convert(r_keypoints, right_features);
	Mat right_features_flat = Mat(right_features).reshape(1, int(right_features.size()));

	//Look around each OF point in the right img for any features that were detected in its area and make a match
	BFMatcher matcher(CV_L2);
	vector<vector<DMatch> > knn_matches;
	//FlannBasedMatcher matcher;
	matcher.radiusMatch(to_find_flat, right_features_flat, knn_matches, 2.0f);
	//check that the found neightboors are unique 
	//throw away neighboors that are too close together, as they may be confusing
	for (size_t i = 0;i<knn_matches.size();i++)
	{
		DMatch _m;
		if (knn_matches[i].size() == 1)
		{
			_m = knn_matches[i][0];
		}
		else if (knn_matches[i].size()>1)
		{
			if (knn_matches[i][0].distance / knn_matches[i][1].distance < 0.7)
			{
				_m = knn_matches[i][0];
			}
			else
			{
				continue; // did not pass ratio test
			}
		}
		else
		{
			continue; // no match
		}
		//prevent duplicates
		set<int> found_in_right_points;//for duplicate prevention
		if (found_in_right_points.find(_m.trainIdx) == found_in_right_points.end())
		{ //the found neighboor was not yet used
		  //we should match it with the orignal indexing of the left point
			_m.queryIdx = to_find_back_index[_m.queryIdx]; //back to original indexing of points for <i_idx>
			matches.push_back(_m);//add this match
			found_in_right_points.insert(_m.trainIdx);
		}
	}
	cout << "Opitical Flow matched:  " << matches.size() << " matches" << endl;
}
void FeatureMatch::RichFeatureMatch()
{
	Mat l_descriptors, r_descriptors;
	//ORB提取特征
	/*Ptr<ORB> orb = ORB::create();
	orb->detectAndCompute(img1, noArray(), l_keypoints, l_descriptors);
	orb->detectAndCompute(img2, noArray(), r_keypoints, r_descriptors);
	BFMatcher matcher(NORM_HAMMING, true);
	matcher.match(l_descriptors, r_descriptors, matches);*/
	//SURF
	Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create();
	detector->setHessianThreshold(400);
	detector->detectAndCompute(img1, Mat(), l_keypoints, l_descriptors);
	detector->detectAndCompute(img2, Mat(), r_keypoints, r_descriptors);
	BFMatcher matcher(NORM_L2);
	matcher.match(l_descriptors, r_descriptors, matches);
	//cout << "SURF matched: " << matches.size() << "matches" << endl;
	MatchesOptimize();//用基础矩阵优化
	cout << "SURF matched(after optimize with using fundamental matrix): " << matches.size() << endl;
}
void FeatureMatch::MatchesOptimize()
{
	if (!(matches.size()&&l_keypoints.size()&&r_keypoints.size())) return;
	/*RANSAC 消除误匹配特征点 主要分为三个部分：
	1）根据matches将特征点对齐,将坐标转换为float类型
	2）使用求基础矩阵方法 findFundamentalMat,得到RansacStatus
	3）根据RansacStatus来将误匹配的点也即RansacStatus[i]=0的点删除*/
	//根据matches将特征点对齐,将坐标转换为float类型
	vector<KeyPoint> L_keypoints, R_keypoints;
	vector<DMatch> Re_matches;
	//坐标转换
	vector<Point2f>l_points, r_points;
	for (size_t i = 0; i<matches.size(); i++)
	{
		l_points.push_back(l_keypoints[matches[i].queryIdx].pt);
		r_points.push_back(r_keypoints[matches[i].trainIdx].pt);
		//这两句话的理解：R_keypoint1是要存储img1中能与img2匹配的特征点，
		//matches中存储了这些匹配点对的img1和img2的索引值
	}
	//利用基础矩阵剔除误匹配点
	vector<uchar> RansacStatus;//mask
	Mat Fundamental = findFundamentalMat(l_points, r_points,  FM_RANSAC, 3, 0.99, RansacStatus);
	int index = 0;
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			L_keypoints.push_back(l_keypoints[i]);
			R_keypoints.push_back(r_keypoints[i]);
			matches[i].queryIdx = index;
			matches[i].trainIdx = index;
			Re_matches.push_back(matches[i]);
			index++;
		}
	}
	if (L_keypoints.size() && R_keypoints.size() && Re_matches.size()) {
		l_keypoints.clear();
		r_keypoints.clear();
		matches.clear();
		l_keypoints = L_keypoints;
		r_keypoints = R_keypoints;
		matches = Re_matches;
	}
	else
	{
		cout << "optimize failed! (matches_optimize(***))" << endl;
		return;
	}
}
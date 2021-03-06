#pragma once
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <set>

using namespace std;
using namespace cv;

class FeatureMatch {
private:
	vector<KeyPoint> l_keypoints, r_keypoints;
	vector<DMatch> matches;
	Mat img1, img2;
public:
	FeatureMatch::FeatureMatch(const Mat l_img, const Mat r_img) 
	{
		l_img.copyTo(img1);
		r_img.copyTo(img2); 
	}
	FeatureMatch::~FeatureMatch()
	{
		l_keypoints.shrink_to_fit();
		//l_keypoints.swap(vector<KeyPoint> ());
		r_keypoints.shrink_to_fit();
		//r_keypoints.swap(vector<KeyPoint>());
		matches.shrink_to_fit();
		//matches.swap(vector<DMatch>());
		img1.release();
		img2.release();
	}
	void FeatureMatch::RichFeatureMatch();//SURF(default) || ORB
	void FeatureMatch::OFFeatureMatch();//Opitical Flow
	void FeatureMatch::MatchesOptimize();
	vector<KeyPoint> getKeypoints_L() {
		return l_keypoints;
	}
	vector<KeyPoint> getKeypoints_R() {
		return r_keypoints;
	}
	vector<DMatch> getMatches() {
		return matches;
	}
};
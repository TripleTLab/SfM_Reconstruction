#pragma once

#include <opencv2/core/core.hpp>

#include <vector>
#include <iostream>
#include <list>
#include <set>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;


struct CloudPoint {
	Point3d pt;
	vector<int> imgpt_for_img;
	double reprojection_error;
};

vector<DMatch> FlipMatches(const vector<DMatch>& matches);
void KeyPointsToPoints(const vector<KeyPoint>& kps, vector<Point2f>& ps);
void PointsToKeyPoints(const vector<Point2f>& ps, vector<KeyPoint>& kps);
vector<Point3d> CloudPointsToPoints(const vector<CloudPoint> cpts);



void drawArrows(Mat& frame, const vector<Point2f>& prevPts, const vector<Point2f>& nextPts, const vector<uchar>& status, const vector<float>& verror, const Scalar& line_color = Scalar(0, 0, 255));
void imshow_250x250(const string& name_, const Mat& patch);

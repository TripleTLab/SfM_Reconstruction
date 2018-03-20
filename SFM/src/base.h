#pragma once

#include "FeatureMatch.h"
#include "CameraCalibration.h"
#include "FileIO.h"

const double focal = 5.0, u0 = 320, v0 = 240, dx = 5.312 / 640, dy = 3.984 / 480, fu = focal / dx, fv = focal / dy;
const double fx = focal * 180 / 25.4, fy = focal * 180 / 25.4;

const Matx33d K = Matx33d(fx, 0, u0,
						  0, fy, v0,
						  0,  0,  1);
const Point2d pp(u0, v0);

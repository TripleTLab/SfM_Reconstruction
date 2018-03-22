/* Epipolar.h */
/* Routines for computing epipolar geometry */

#ifndef __epipolar_h__
#define __epipolar_h__

#include <vector.h>

#include "keys.h"

/* 从一组给定的匹配点预估本质矩阵 */
std::vector<int> EstimateEMatrix(const std::vector<Keypoint> &k1,
				 const std::vector<Keypoint> &k2,
				 std::vector<KeypointMatch> matches,
				 int num_trials, double threshold,
				 double f1, double f2,
                                 double *E, double *F);

/* 从一组给定的点匹配中预估相对姿态 */
int EstimatePose5Point(const std::vector<Keypoint> &k1,
                       const std::vector<Keypoint> &k2,
                       std::vector<KeypointMatch> matches,
                       int num_trials, double threshold,
                       double *K1, double *K2,
                       double *R, double *t);

/* 从一组给定的点匹配中预估 F 矩阵 */
std::vector<int> EstimateFMatrix(const std::vector<Keypoint> &k1,
				 const std::vector<Keypoint> &k2,
				 std::vector<KeypointMatch> matches,
				 int num_trials, double threshold,
				 double *F, bool essential = false);

std::vector<int> EstimateFMatrix(const std::vector<KeypointWithDesc> &k1,
				 const std::vector<KeypointWithDesc> &k2,
				 std::vector<KeypointMatch> matches,
				 int num_trials, double threshold,
				 double *F, bool essential = false);

#endif /* __epipolar_h__ */

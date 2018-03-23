/* Distortion.h */

#ifndef __distortion_h__
#define __distortion_h__

#include "sfm.h"
#include "vector.h"

void InvertDistortion(int n_in, int n_out, double r0, double r1,
                      double *k_in, double *k_out);

v2_t UndistortNormalizedPoint(v2_t p, camera_params_t c);

#endif /* __distortion_h__ */

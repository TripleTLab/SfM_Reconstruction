


#ifndef __bundle_h__
#define __bundle_h__

#include "ImageData.h"
#include "Geometry.h"

#include "sfm.h"


double ComputeRayAngle(v2_t p, v2_t q,
		       const camera_params_t &cam1,
		       const camera_params_t &cam2);


bool CheckCheirality(v3_t p, const camera_params_t &camera);

void GetIntrinsics(const camera_params_t &camera, double *K);

double GetCameraDistance(camera_params_t *c1, camera_params_t *c2);


void FixIntrinsics(double *P, double *K, double *R, double *t);

void InitializeCameraParams(const ImageData &data, camera_params_t &camera);

#endif 

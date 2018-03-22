/* BundleAdd.h */

#ifndef __bundle_add_h__
#define __bundle_add_h__

#include "vector.h"
#include "sfm.h"

v3_t Triangulate(v2_t p, v2_t q,
                 camera_params_t c1, camera_params_t c2,
                 double &proj_error, bool &in_front, double &angle,
                 bool explicit_camera_centers);

#endif /* __bundle_add_h__ */

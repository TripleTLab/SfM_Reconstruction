/* Register.h */
/* 计算图片之间的关系 */

#ifndef __register_h__
#define __register_h__

#include <vector>



#include "keys.h"

enum MotionModel {
    MotionRigid,
    MotionHomography,
};

/* 预估两组关键点之间的转换 */
std::vector<int> EstimateTransform(const std::vector<Keypoint> &k1,
				   const std::vector<Keypoint> &k2,
				   const std::vector<KeypointMatch> &matches,
				   MotionModel mm,
				   int nRANSAC, double RANSACthresh,
				   double *Mout);



#endif /* __register_h__ */

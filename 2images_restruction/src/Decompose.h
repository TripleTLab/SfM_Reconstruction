


#ifndef __decompose_h__
#define __decompose_h__

#include "vector.h"


bool DecomposeHomography(double *H, double f1, double f2,
                         double *Ra, double *ta,
                         double *Rb, double *tb, v2_t p, v2_t q);

bool ComputeFundamentalMatrix(double f1, double f2,
                              double *R2, double *t2, double *F);

#endif 

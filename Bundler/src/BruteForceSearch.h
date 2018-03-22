/* BruteForceSearch.h */
//暴力搜索最近点对数据结构头文件

#ifndef __brute_force_search_h__
#define __brute_force_search_h__

#include <stdlib.h>
#include "vector.h"

class BruteForceSearch
{
public:
    BruteForceSearch() : m_num_points(0), m_points(NULL) { }
    BruteForceSearch(int n, v3_t *pts);
    ~BruteForceSearch() { if (m_points != NULL) delete [] m_points; }

    void GetClosestPoints(v3_t query, int num_points,
			  int *idxs, double *dists);

    int m_num_points;
    v3_t *m_points;
};

#endif /* __brute_force_search_h__ */

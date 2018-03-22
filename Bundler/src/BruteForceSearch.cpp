/* BruteForceSearch.cpp */
//暴力搜索最近点对数据结构

#include "BruteForceSearch.h"

#include "qsort.h"
#include "vector.h"

BruteForceSearch::BruteForceSearch(int n, v3_t *pts)
{
    m_num_points = n;
    m_points = new v3_t[n];

    for (int i = 0; i < n; i++)
	m_points[i] = pts[i];
}

void BruteForceSearch::GetClosestPoints(v3_t query, int num_points,
					int *idxs, double *dists)
{
    double *dists_all = new double[m_num_points];

    for (int i = 0; i < m_num_points; i++) {
	dists_all[i] = v3_magsq(v3_sub(query, m_points[i]));
    }

    int *perm = new int[m_num_points];
    qsort_ascending();
    qsort_perm(m_num_points, dists_all, perm);

    for (int i = 0; i < num_points; i++) {
	idxs[i] = perm[i];
	dists[i] = dists_all[i];
    }

    delete [] dists_all;
    delete [] perm;
}

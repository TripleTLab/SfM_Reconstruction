#include <float.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "BoundingBox.h"

#include "defines.h"

void BoundingBox::Print()
{
    printf("[(%0.3f, %0.3f) x (%0.3f, %0.3f)]\n",
	   m_xmin, m_ymin, m_xmax, m_ymax);

}

/* 返回边框面积 */
double BoundingBox::Area()
{
    return (m_xmax - m_xmin) * (m_ymax - m_ymin);
}

BoundingBox BoundingBox::Intersect(const BoundingBox &bbox) const
{
	return BoundingBox(MAX(m_xmin, bbox.m_xmin), MAX(m_ymin, bbox.m_ymin),
					   MIN(m_xmax, bbox.m_xmax), MIN(m_ymax, bbox.m_ymax));
}

/* 返回真如果点在边框内 */
bool BoundingBox::Contains(double x, double y)
{
    return (x >= m_xmin && x <= m_xmax && y >= m_ymin && y <= m_ymax);
}

bool BoundingBox::Contains(const BoundingBox &bbox)
{
    return (bbox.m_xmin >= m_xmin &&
	    bbox.m_xmax <= m_xmax &&
	    bbox.m_ymin >= m_ymin &&
	    bbox.m_ymax <= m_ymax);
}

void BoundingBox::Scale(double scale)
{
    m_xmin *= scale;
    m_xmax *= scale;
    m_ymin *= scale;
    m_ymax *= scale;
}

/* 对一组点建立边框 */
BoundingBox CreateBoundingBox(const std::vector<v2_t> &points)
{
    int num_points = (int) points.size();
    BoundingBox bb;

    bb.m_xmin = DBL_MAX;
    bb.m_xmax = -DBL_MAX;
    bb.m_ymin = DBL_MAX;
    bb.m_ymax = -DBL_MAX;

    if (num_points == 0) {
	printf("[CreateBoundingBox] No points given!\n");
	return bb;
    }

    for (int i = 0; i < num_points; i++) {
	bb.m_xmin = MIN(bb.m_xmin, Vx(points[i]));
	bb.m_xmax = MAX(bb.m_xmax, Vx(points[i]));
	bb.m_ymin = MIN(bb.m_ymin, Vy(points[i]));
	bb.m_ymax = MAX(bb.m_ymax, Vy(points[i]));
    }

    return bb;
}

/* 合并两个边框 */
BoundingBox BoundingBoxUnion(const BoundingBox &bb1, const BoundingBox &bb2)
{
    BoundingBox bb;
    bb.m_xmin = MIN(bb1.m_xmin, bb2.m_xmin);
    bb.m_xmax = MAX(bb1.m_xmax, bb2.m_xmax);

    bb.m_ymin = MIN(bb1.m_ymin, bb2.m_ymin);
    bb.m_ymax = MAX(bb1.m_ymax, bb2.m_ymax);

    return bb;
}

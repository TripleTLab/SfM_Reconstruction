/* BoundingBox.h */

#ifndef __bounding_box_h__
#define __bounding_box_h__

#include <vector>
#include "vector.h"

class BoundingBox {
public:
    BoundingBox() { }
    BoundingBox(double xmin, double ymin, double xmax, double ymax) :
	m_xmin(xmin), m_xmax(xmax), m_ymin(ymin), m_ymax(ymax) { }
    BoundingBox(double x, double y) :
	m_xmin(x), m_xmax(x), m_ymin(y), m_ymax(y) { }

    //返回边界框的区域
    double Area();

	BoundingBox Intersect(const BoundingBox &bbox) const;

    bool Contains(double x, double y);
    bool Contains(const BoundingBox &bbox);

    void Scale(double scale);
    void Print();

    double Width()  { return m_xmax - m_xmin; }
    double Height() { return m_ymax - m_ymin; }

    double m_xmin, m_xmax;
    double m_ymin, m_ymax;
};

//为一组点创建边界框
BoundingBox CreateBoundingBox(const std::vector<v2_t> &points);

//返回两个边界框的集合
BoundingBox BoundingBoxUnion(const BoundingBox &bb1, const BoundingBox &bb2);

#endif /* __bounding_box_h__ */

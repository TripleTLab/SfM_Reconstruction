/* ParameterBound.h */

#ifndef __parameter_bound_h__
#define __parameter_bound_h__

class ParameterBound
{
public:
    ParameterBound() { }
    ParameterBound(float min_x, float min_y, float max_x, float max_y) :
	m_min_x(min_x), m_min_y(min_y), m_max_x(max_x), m_max_y(max_y)
    { }

    float m_min_x, m_min_y;
    float m_max_x, m_max_y;
};

#endif /* __parameter_bound_h__ */

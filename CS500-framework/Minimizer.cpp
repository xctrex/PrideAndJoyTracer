#include "stdafx.h"
#include "Minimizer.h"

float Minimizer::minimumOnObject(Shape* obj) {
    Intersection intersection;
    if (obj->Intersect(m_ray, intersection))
    {
        if (intersection.t < m_minimumIntersection.t && intersection.t > FLT_EPSILON)
        {
            m_minimumIntersection = intersection;
        }

        return m_minimumIntersection.t;
    }

    return INF;
}

float Minimizer::minimumOnVolume(const Eigen::AlignedBox<float, 3>& box)
{
    Eigen::Vector3f blf = box.corner(Eigen::AlignedBox<float, 3>::BottomLeftFloor);
    Eigen::Vector3f trc = box.corner(Eigen::AlignedBox<float, 3>::TopRightCeil);
    Box b(vec3(blf.x(), blf.y(), blf.z()), vec3(trc.x() - blf.x(), trc.y() - blf.y(), trc.z() - blf.z()));

    Intersection intersection;
    if (b.Intersect(m_ray, intersection))
    {
        return intersection.t;
    }

    return INF;
}

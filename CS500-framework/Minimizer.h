#pragma once
#include "stdafx.h"
#include "Ray.h"
#include "Shape.h"

struct Minimizer
{
public:
    typedef float Scalar;

    Minimizer(const Ray& r)
        : m_ray(r)
    {
        m_minimumIntersection.t = INF;
    }

    // Called by BVMinimize to intersect the ray with a Shape.  This
    // should return the intersection t, but should also track
    // the minimum t and it's corresponding intersection info.
    // Return INF to indicate no intersection.
    float minimumOnObject(Shape* obj);


    // Called by BVMinimize to intersect the ray with a Box3d and
    // returns the t value.  This should be identical to the already
    // written box (3 slab) intersection.
    // Return INF to indicate no intersection.
    float minimumOnVolume(const Eigen::AlignedBox<float, 3>& box);

    Ray m_ray;
    Intersection m_minimumIntersection;
};
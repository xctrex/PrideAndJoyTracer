#include "stdafx.h"
#include "Shape.h"

Eigen::AlignedBox<float, 3> bounding_box(const Shape* obj)
{
    return obj->BoundingBox();
}

bool Sphere::Intersect(const Ray& ray, Intersection& intersection) const
{
    vec3 D = ray.Q - m_center;
    double a = glm::dot(ray.R, ray.R);
    double b = 2.0 * glm::dot(ray.R, D);
    double c = glm::dot(D, D) - glm::pow2(m_radius);
    double discriminant = glm::pow2(b) - 4.0 * a * c;
    if (discriminant > 0.0)
    {
        discriminant = sqrt(discriminant);
        double tMinus = (-b - discriminant) / (2.0 * a);
        double tPlus = (-b + discriminant) / (2.0 * a);

        // Evaluate the smallest positive t-value
        if (tMinus < 0.0 && tPlus < 0.0)
        {
            return false;
        }
        else if (tMinus < 0.0)
        {
            EvalIntersectionAtT(ray, tPlus, intersection);
        }
        else if (tPlus < 0.0)
        {
            EvalIntersectionAtT(ray, tMinus, intersection);
        }
        else
        {
            EvalIntersectionAtT(ray, glm::min(tMinus, tPlus), intersection);
        }
        return true;
    }
    return false;
}

void Sphere::EvalIntersectionAtT(const Ray& ray, double t, Intersection& intersection) const
{
    intersection.t = t;
    intersection.position = ray.Eval(intersection.t);
    intersection.normal = intersection.position - m_center;
    intersection.object = static_cast<const Shape*>(this);
}

Eigen::AlignedBox<float, 3> Sphere::BoundingBox() const
{
    // Bottom left floor
    vec3 blf = m_center - vec3(m_radius, m_radius, m_radius);
    // Top right corner
    vec3 trc = m_center + vec3(m_radius, m_radius, m_radius);    

    return Eigen::AlignedBox<float, 3>(Eigen::Vector3f(blf.x, blf.y, blf.z), Eigen::Vector3f(trc.x, trc.y, trc.z));
}

bool Plane::Intersect(const Ray& ray, Intersection& intersection) const
{
    return false;
}

T_N tnMax(const T_N& tn0, const T_N& tn1)
{
    if (tn0.t > tn1.t)
    {
        return tn0;
    }
    return tn1;
}

T_N tnMin(const T_N& tn0, const T_N& tn1)
{
    if (tn0.t < tn1.t)
    {
        return tn0;
    }
    return tn1;
}

bool Slab::Intersect(const Ray& ray, std::pair<T_N, T_N>& intersection) const
{
    double nDotR = glm::dot(m_N, ray.R);
    // if nDotR != 0
    if (glm::abs(nDotR) > FLT_EPSILON)
    {
        T_N t0(-(m_d0 + glm::dot(m_N, ray.Q)) / nDotR, -m_N);
        T_N t1(-(m_d1 + glm::dot(m_N, ray.Q)) / nDotR, m_N);
        if (t0.t < t1.t)
        {
            intersection = std::pair<T_N, T_N>(t0, t1);
        }
        else
        {
            intersection = std::pair<T_N, T_N>(t1, t0);
        }
        return true;
    }
    else
    {
        double s0 = glm::dot(m_N, ray.Q) + m_d0;
        double s1 = glm::dot(m_N, ray.Q) + m_d1;
        // If the signs of s0 and s1 differ
        if (s0 < 0.0 && s1 > 0.0 || s0 > 0.0 && s1 < 0.0)
        {
            intersection = std::pair<T_N, T_N>(T_N(0.0, m_N), T_N(FLT_MAX, -m_N));
            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

bool SlabShape::Intersect(const Ray& ray, T_N& tn) const
{
    std::pair<T_N, T_N> result(T_N(0.0, vec3(0.0, 0.0, 0.0)), T_N(FLT_MAX, vec3(0.0, 0.0, 0.0)));
    for (size_t i = 0; i < m_slabs.size(); ++i)
    {
        std::pair<T_N, T_N> interval(T_N(0.0, vec3(0.0, 0.0, 0.0)), T_N(FLT_MAX, vec3(0.0, 0.0, 0.0)));
        // If there is an intersection between the ray and this slab
        if (m_slabs[i].Intersect(ray, interval))
        {
            result.first = tnMax(result.first, interval.first);
            result.second = tnMin(result.second, interval.second);
        }
        else
        {
            // If there is even one slab with no intersection, then there is no intersection for the whole shape
            return false;
        }
    }
    // This check catches "just off the corner" cases
    if (result.first.t > result.second.t)
    {
        return false;
    }    

    // Evaluate the smallest positive t-value
    if (result.first.t < 0.0 && result.second.t < 0.0)
    {
        return false;
    }
    else if (result.first.t < 0.0)
    {
        tn = result.second;
    }
    else if (result.second.t < 0.0)
    {
        tn = result.first;
    }
    else
    {
        tn = tnMin(result.first, result.second);
    }
    return true;
}

bool Box::Intersect(const Ray& ray, Intersection& intersection) const
{
    T_N tn;
    if (m_slabBox.Intersect(ray, tn))
    {
        intersection.t = tn.t;
        intersection.normal = tn.n;
        intersection.position = ray.Eval(intersection.t);
        intersection.object = this;
        return true;
    }
    return false;
}

Eigen::AlignedBox<float, 3> Box::BoundingBox() const
{
    return Eigen::AlignedBox<float, 3>(Eigen::Vector3f(m_bottomLeftFloor.x, m_bottomLeftFloor.y, m_bottomLeftFloor.z), Eigen::Vector3f(m_topRightCeiling.x, m_topRightCeiling.y, m_topRightCeiling.z));
}

bool Triangle::Intersect(const Ray& ray, Intersection& intersection) const
{
    vec3 E1 = m_v1 - m_v0;
    vec3 E2 = m_v2 - m_v0;
    vec3 p = glm::cross(ray.R, E2);
    double d = glm::dot(p, E1);
    // If ray is parallel to triangle
    if (glm::abs(d) < FLT_EPSILON)
    {
        return false;
    }
    vec3 S = ray.Q - m_v0;
    double u = glm::dot(p, S) / d;
    // If ray intersects plane, but outside E2 edge
    if (u < 0 || u > 1)
    {
        return false;
    }
    vec3 q = glm::cross(S, E1);
    double v = glm::dot(ray.R, q) / d;
    // If ray intersects plane, but outside other edges
    if (v < 0 || u + v > 1)
    {
        return false;
    }
    double t = glm::dot(E2, q) / d;
    // Ray's negative half intersects triangle
    if (t < 0)
    {
        return false;
    }

    intersection.t = t;
    intersection.position = ray.Eval(intersection.t);
    intersection.normal = glm::cross(E2, E1);
    intersection.object = this;

    return true;
}

Eigen::AlignedBox<float, 3> Triangle::BoundingBox() const
{
    // Bottom left floor
    vec3 blf = vec3(glm::min(m_v0.x, m_v1.x, m_v2.x), glm::min(m_v0.y, m_v1.y, m_v2.y), glm::min(m_v0.z, m_v1.z, m_v2.z));
    // Top right corner
    vec3 trc = vec3(glm::max(m_v0.x, m_v1.x, m_v2.x), glm::max(m_v0.y, m_v1.y, m_v2.y), glm::max(m_v0.z, m_v1.z, m_v2.z));

    return Eigen::AlignedBox<float, 3>(Eigen::Vector3f(blf.x, blf.y, blf.z), Eigen::Vector3f(trc.x, trc.y, trc.z));
}

bool Cylinder::Intersect(const Ray& ray, Intersection& intersection) const
{
    vec3 zAxis(0.0, 0.0, 1.0);
    vec3 rotationAxis = glm::cross(m_axis, zAxis);
    quat q;
    
    // If m_axis is parallel to the z-axis
    if (glm::l2Norm(rotationAxis) < FLT_EPSILON)
    {
        q = quat(1.0, 0.0, 0.0, 0.0);
    }
    else
    {
        q = glm::angleAxis(glm::acos(glm::dot(m_axis, zAxis) / glm::l2Norm(m_axis)), glm::normalize(rotationAxis));
    }

    Ray transformedRay = ray;
    transformedRay.Q = glm::rotate(q, ray.Q - m_basePoint);
    transformedRay.R = glm::rotate(q, ray.R);

    // Three intervals:
    // 1. The ray itself
    std::pair<double, double> t0(0.0, FLT_MAX);

    // 2. Ray's intersection with the slab that caps the cylinder's top and bottom
    Slab capSlab(zAxis, 0.0, -glm::l2Norm(m_axis));
    std::pair<T_N, T_N> capSlabInterval(T_N(0.0, vec3(0.0, 0.0, 0.0)), T_N(FLT_MAX, vec3(0.0, 0.0, 0.0)));
    if (!capSlab.Intersect(transformedRay, capSlabInterval))
    {
        return false;
    }

    // 3. Ray's intersection with infinite cylinder
    std::pair<double, double> cylinderInterval(0.0, FLT_MAX);
    double a = glm::pow2(transformedRay.R.x) + glm::pow2(transformedRay.R.y);
    double b = 2.0 * (transformedRay.R.x * transformedRay.Q.x + transformedRay.R.y * transformedRay.Q.y);
    double c = glm::pow2(transformedRay.Q.x) + glm::pow2(transformedRay.Q.y) - glm::pow2(m_radius);

    double descriminant = glm::pow2(b) - 4.0 * a * c;
    if (descriminant < 0.0)
    {
        return false;
    }
    descriminant = sqrt(descriminant);

    cylinderInterval.first = (-b - descriminant) / (2.0 * a);
    cylinderInterval.second = (-b + descriminant) / (2.0 * a);

    // Determine if it intersects both the cylinder and the slab at a point at which the cylinder is within the slab
    std::pair<double, double> result;
    result.first = glm::max(t0.first, capSlabInterval.first.t, cylinderInterval.first);
    result.second = glm::min(t0.second, capSlabInterval.second.t, cylinderInterval.second);
    if (result.first > result.second)
    {
        return false;
    }

    intersection.t = result.first;
    intersection.position = ray.Eval(intersection.t);
    
    // Get the intersection normal if it's on the cap
    if (capSlabInterval.first.t > intersection.t - FLT_EPSILON && capSlabInterval.first.t < intersection.t + FLT_EPSILON)
    {
        intersection.normal = capSlabInterval.first.n;
    }
    // Get the intersection normal from the cylinder
    else
    {
        vec3 M = transformedRay.Eval(intersection.t);
        intersection.normal = vec3(M.x, M.y, 0);
    }
    // Transform the normal back from the z-axis-aligned space
    intersection.normal = glm::normalize(glm::rotate(glm::conjugate(q), intersection.normal));

    intersection.object = this;
    return true;
}

Eigen::AlignedBox<float, 3> Cylinder::BoundingBox() const
{
    vec3 otherPoint = m_basePoint + m_axis;
    // Bottom left floor
    vec3 blf = vec3(glm::min(m_basePoint.x, otherPoint.x), glm::min(m_basePoint.y, otherPoint.y), glm::min(m_basePoint.z, otherPoint.z));
    // Top right corner
    vec3 trc = vec3(glm::max(m_basePoint.x, otherPoint.x), glm::max(m_basePoint.y, otherPoint.y), glm::max(m_basePoint.z, otherPoint.z));

    // Account for the radius
    // This is not a perfectly tight bounding box, but it will do
    blf -= vec3(m_radius, m_radius, m_radius);
    trc += vec3(m_radius, m_radius, m_radius);

    return Eigen::AlignedBox<float, 3>(Eigen::Vector3f(blf.x, blf.y, blf.z), Eigen::Vector3f(trc.x, trc.y, trc.z));
}

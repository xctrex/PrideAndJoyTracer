#pragma once
#include "stdafx.h"
#include "Ray.h"
#include "Realtime.h"

struct Material
{
    vec3 Kd;
    vec3 Ks;
    vec3 Kt;
    vec3 emitted;
    double roughness;
    double indexOfRefraction;
    bool isLight;
};

class Intersection;

// Calculates the effect of light leaving one point and stringing another point
double GeometryFactor(const Intersection &a, const Intersection &b);
// Samples a cone of random vectors around V
vec3 SampleCone(const vec3 v, double cosTheta, double phi);

class Shape
{
public:
    Shape()
    {}

    virtual bool Intersect(const Ray& ray, Intersection& intersection) const = 0;
    virtual Eigen::AlignedBox<float, 3> BoundingBox() const = 0;
    vec3 Kd() const { return m_material.Kd; }
    vec3 Ks() const { return m_material.Ks; }
    vec3 Kt() const { return m_material.Kt; }
    double Roughness() const { return m_material.roughness; }
    double IndexOfRefraction() const { return m_material.indexOfRefraction; }
    bool IsLight() const { return m_material.isLight; }
    Material m_material;
};

Eigen::AlignedBox<float, 3> bounding_box(const Shape* obj);

class Sphere : public Shape
{
public:
    Sphere(vec3 center, double radius)
        : m_center(center)
        , m_radius(radius)
    {}

    bool Intersect(const Ray& ray, Intersection& intersection) const;
    void EvalIntersectionAtT(const Ray& ray, double t, Intersection& intersection) const;
    Eigen::AlignedBox<float, 3> BoundingBox() const;

    vec3 m_center;
    double m_radius;
};

class Plane : public Shape
{
public:
    Plane(vec3 normal, vec3 point)
        : m_N(normal)
        , m_d0(point)
    {}

    bool Intersect(const Ray& ray, Intersection& intersection) const;
    Eigen::AlignedBox<float, 3> BoundingBox() const;

    vec3 m_N;
    vec3 m_d0;
};

struct T_N
{
    T_N()
        : t(-1.0)
        , n(vec3(0.0, 0.0, 0.0))
    {}

    T_N(double time, vec3 normal)
        : t(time)
        , n(normal)
    {}

    double t;
    vec3 n;
};

T_N tnMax(const T_N& tn0, const T_N& tn1);

T_N tnMin(const T_N& tn0, const T_N& tn1);


class Slab
{
public:
    Slab(vec3 normal, double point0, double point1)
        : m_N(normal)
        , m_d0(point0)
        , m_d1(point1)
    {}
       
    bool Intersect(const Ray& ray, std::pair<T_N, T_N>& intersection) const;

    vec3 m_N;
    double m_d0;
    double m_d1;
};

class SlabShape
{
public:
    bool Intersect(const Ray& ray, T_N& t) const;

    std::vector<Slab> m_slabs;
};

class Box : public Shape
{
public:
    Box(vec3 corner, vec3 diagonalVector)
    {
        vec3 otherCorner = corner + diagonalVector;
        m_bottomLeftFloor = vec3(glm::min(corner.x, otherCorner.x), glm::min(corner.y, otherCorner.y), glm::min(corner.z, otherCorner.z));
        m_topRightCeiling = vec3(glm::max(corner.x, otherCorner.x), glm::max(corner.y, otherCorner.y), glm::max(corner.z, otherCorner.z));
        // Make a slab shape
        // Slab with normals in the x direction
        m_slabBox.m_slabs.push_back(Slab(vec3(1.0, 0.0, 0.0), -m_bottomLeftFloor.x, -m_topRightCeiling.x));

        // Slab with normals in the y direction
        m_slabBox.m_slabs.push_back(Slab(vec3(0.0, 1.0, 0.0), -m_bottomLeftFloor.y, -m_topRightCeiling.y));

        // Slab with normals in the z direction
        m_slabBox.m_slabs.push_back(Slab(vec3(0.0, 0.0, 1.0), -m_bottomLeftFloor.z, -m_topRightCeiling.z));
    }

    bool Intersect(const Ray& ray, Intersection& intersection) const;
    Eigen::AlignedBox<float, 3> BoundingBox() const;

    SlabShape m_slabBox;
    vec3 m_bottomLeftFloor;
    vec3 m_topRightCeiling;
};

class Cylinder : public Shape
{
public :
    Cylinder(vec3 basePoint, vec3 axis, double radius)
        : m_basePoint(basePoint)
        , m_axis(axis)
        , m_radius(radius)
    {}

    bool Intersect(const Ray& ray, Intersection& intersection) const;
    Eigen::AlignedBox<float, 3> BoundingBox() const;

    // Base point
    vec3 m_basePoint;
    // Axis
    vec3 m_axis;
    // radius
    double m_radius;
};

inline double tripleProduct(vec3 a, vec3 b, vec3 c)
{
    return glm::dot(glm::cross(a, b), c);
}

class Triangle : public Shape
{
public:
    Triangle(vec3 v0, vec3 v1, vec3 v2)
        : m_v0(v0)
        , m_v1(v1)
        , m_v2(v2)
    {}

    bool Intersect(const Ray& ray, Intersection& intersection) const;
    Eigen::AlignedBox<float, 3> BoundingBox() const;

    vec3 m_v0;
    vec3 m_v1;
    vec3 m_v2;
};

enum class RadiationType
{
    Diffuse,
    Reflection,
    Transmission
};

class Intersection
{
public:
    Intersection();
    bool IsValid() const { return t < FLT_MAX - FLT_EPSILON; }
    void SampleBRDF(const vec3 wo, vec3 &wi, RadiationType &type) const;
    double PDFBRDF(const vec3 wo, const vec3 wi) const;
    vec3 EvaluateBRDF(const vec3 wo, const vec3 wi) const;
    vec3 Kd() const { return object->Kd(); }
    vec3 Ks() const { return object->Ks(); }
    vec3 Kt() const { return object->Kt(); }
    double Roughness() const { return object->Roughness(); }
    double IndexOfRefraction() const { return object->IndexOfRefraction(); }
    bool IsLight() const { return object->IsLight(); }
    double t = FLT_MAX;
    vec3 normal;
    vec3 position;
    const Shape *object;
private:
    double Pd(const vec3 wo, const vec3 wi) const;
    double Pr(const vec3 wo, const vec3 wi) const;
    double Pt(const vec3 wo, const vec3 wi) const;
    vec3 Ed(const vec3 wo, const vec3 wi) const;
    vec3 Er(const vec3 wo, const vec3 wi) const;
    vec3 Et(const vec3 wo, const vec3 wi) const;

    double D(const vec3 m) const;
    double G1(double nDotV) const;
    double G(const vec3 wo, const vec3 wi, const vec3 m) const;
    vec3 F(double vDotH) const;

    double m_probabilityDiffuse;
    double m_probabilityReflection;
    double m_probabilityTransmission;
    double m_no;
    double m_ni;
};

double Characteristic(double d);
double tanTheta(double vDotN);
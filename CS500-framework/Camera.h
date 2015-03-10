#pragma once
#include "stdafx.h"
#include "Ray.h"
class Camera
{
public:
    Camera()
        : eyePos(0.f, 0.f, 0.f)
        , viewOrientation()
        , ry(.2f)
        , m_Width(300)
        , m_Height(200)
    {}

    Camera(vec3 pos, quat orientation, double ratio, double w, double h)
        : eyePos(pos)
        , viewOrientation(orientation)
        , ry(ratio)
        , m_Width(w)
        , m_Height(h)
    {
        double rx = ry * m_Width / m_Height;
        X = glm::rotate(viewOrientation, vec3(rx, 0.0, 0.0));
        Y = glm::rotate(viewOrientation, vec3(0.0, ry, 0.0));
        Z = glm::rotate(viewOrientation, vec3(0.0, 0.0, -1.0));
    }

    vec3 eyePos;
    quat viewOrientation;
    double ry;
    double m_Width;
    double m_Height;

    Ray CalculateRayAtPixel(double x, double y)
    {
        double dx = 2.0 * (x + 0.5) / m_Width - 1.0;
        double dy = 2.0 * (y + 0.5) / m_Height - 1.0;

        return Ray(eyePos, dx * X + dy * Y + Z);     
    }

private:
    vec3 X;
    vec3 Y;
    vec3 Z;
};
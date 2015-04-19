#pragma once
#include "stdafx.h"

class Ray
{
public:
    Ray()
    : Q(0.0, 0.0, 0.0)
    , R(0.0, 0.0, 0.0)
    {};

    Ray(vec3 q, vec3 r)
        : Q(q)
        , R(r)
    {}
    
    vec3 Eval(double t) const
    {
        return Q + t * R;
    }

    vec3 GetR() const{ return R; }

    vec3 Q;
    vec3 R;
};
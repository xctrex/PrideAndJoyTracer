#pragma once

// The vec* types are vectors of DOUBLES, since most raytracing
// calculations should be done with doubles.
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
typedef glm::dvec4 vec4;
typedef glm::dvec3 vec3;
typedef glm::dvec2 vec2;
typedef glm::dquat quat;


#include <Eigen/StdVector>
#include <Eigen_unsupported/Eigen/BVH>

// Forward declaration of shape

const float INF = std::numeric_limits<float>::max();
//typedef Eigen::AlignedBox<float, 3> Box3d;
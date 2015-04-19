///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#pragma once
#include "stdafx.h"

////////////////////////////////////////////////////////////////////////////////
// Scene
#include "Camera.h"
#include "Realtime.h"
#include "Shape.h"
#include "Timer.h"

enum Antialiasing
{
    Off,
    On,
    Jittering
};

class Scene {
public:

    Scene() 
        : m_ambientColor(vec3(0.0, 0.0, 0.0))
    { 
        m_currentMaterial.Kd = vec3(0.0, 0.0, 0.0);
        m_currentMaterial.Ks = vec3(0.0, 0.0, 0.0);
        m_currentMaterial.roughness = 1.0;
        m_currentMaterial.isLight = false;
        m_RealTime = new Realtime(); 
    }

    int m_Width, m_Height;
    int m_arbitraryFlag = 0;
    int m_pixelX = 0;
    int m_pixelY = 0;
    bool m_singlePixel = false;
    bool m_isRealTime = false;
    bool m_usingBVH = true;
    bool m_nextShapeIsLight = false;
    Camera m_Camera;
    Material m_currentMaterial;
    vec3 m_ambientColor;
    Antialiasing m_Antialiasing = Off;
    int m_AntialiasingN = 3;
    Timer m_Timer;

    Eigen::KdBVH<float, 3, Shape*> m_kdBVH;

    Realtime* m_RealTime;         // Remove this (realtime stuff)

    double D(double mDotN, double roughness) const;
    double G(vec3 wi, vec3 wo, vec3 m, vec3 N, double roughness) const;
    vec3 F(double d, vec3 Ks) const;
    vec3 Lighting(const vec3 eyePos, const Intersection& intersection, int recursionLevel) const;
    vec3 GetColor(const Ray& ray) const;
    void CastRayInScene(const Ray& ray, Intersection& closestIntersection) const;

    void PushBackShape(Shape * s);

    void BuildKdTree();

    vec3 GetVertex(std::vector<float>* pnt, int index);

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::string c,
                 const std::vector<double> f,
                 const std::vector<std::string> strings);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::vector<double>& f, const std::string path);
    // As ReadAssimpFile parses the information from the model file,
    // it will call these methods:
    void createRealtimeMaterial();
    void setTexture(const std::string path);
    void setKd(const glm::vec3 c);
    void setAlpha(const float a);
    void triangleMesh(std::vector<float>* pnt,
                      std::vector<float>* nrm,
                      std::vector<float>* tex,
                      std::vector<float>* tan,
                      std::vector<unsigned int>* tris);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(vec3* image, const int pass);

    std::vector<Shape*> m_Objects;
    std::vector<Shape*> m_Lights;
};

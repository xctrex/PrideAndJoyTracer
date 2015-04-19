//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include <math.h>
#include <string>
#include <fstream>
#include <vector>

const double PI = 3.14159;
const double indexOfRefractionAir = 1.0;

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 prng;
std::uniform_real_distribution<> U01random(0.0, 1.0);
// Call U01random(prng) to get a uniformly distributed random number in [0,1].

#include "raytrace.h"
#include "realtime.h"
#include "Shape.h"
#include "Minimizer.h"

void Scene::createRealtimeMaterial() { m_RealTime->createRealtimeMaterial(); }
void Scene::setTexture(const std::string path) { m_RealTime->setTexture(path); }
void Scene::setKd(const glm::vec3 c) { m_RealTime->setKd(c); }
void Scene::setAlpha(const float a) { m_RealTime->setAlpha(a); }
void Scene::triangleMesh(std::vector<float>* pnt,
                  std::vector<float>* nrm,
                  std::vector<float>* tex,
                  std::vector<float>* tan,
                  std::vector<unsigned int>* tris) 
{
    if (m_isRealTime)
    {
        m_RealTime->triangleMesh(pnt, nrm, tex, tan, tris);
    }
    else
    {
        for (size_t i = 0; i < tris->size(); i += 3)
        {
            vec3 v0 = GetVertex(pnt, (*tris)[i]);
            vec3 v2 = GetVertex(pnt, (*tris)[i + 1]);
            vec3 v1 = GetVertex(pnt, (*tris)[i + 2]);
            bool tempNextShapeIsLight = m_nextShapeIsLight;
            PushBackShape(new Triangle(v0, v1, v2));
            // A triangle doesn't count as a whole shape (it's only part of a mesh)
            // so keep the status of m_nextShapeIsLight consistent until the whold mesh
            // has been processed
            m_nextShapeIsLight = tempNextShapeIsLight;
        }
        m_nextShapeIsLight = false;
    }
}

vec3 Scene::GetVertex(std::vector<float>* pnt, int index)
{
    return vec3((*pnt)[index * 4], (*pnt)[index * 4 + 1], (*pnt)[index * 4 + 2]);
}

void Scene::Command(const std::string c, const std::vector<double> f, const std::vector<std::string> strings)
{
    if (c == "screen")
    {
        // syntax: screen width height
        if (m_isRealTime)
        {
            m_RealTime->setScreen(int(f[0]), int(f[1]));
        }
        m_Width = int(f[0]);
        m_Height = int(f[1]);
        m_Camera.m_Width = m_Width;
        m_Camera.m_Height = m_Height;
    }

    else if (c == "disableBVH")
    {
        m_usingBVH = false;
    }

    else if (c == "camera")
    {
        // syntax: camera x y z   qw qx qy qz   ry
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        if (m_isRealTime)
        {
            m_RealTime->setCamera(glm::vec3(f[0], f[1], f[2]), glm::quat(f[3], f[4], f[5], f[6]), f[7]);
        }
        else
        {
            m_Camera = Camera(vec3(f[0], f[1], f[2]), quat(f[3], f[4], f[5], f[6]), f[7], m_Width, m_Height);
        }
    }

    else if (c == "ambient") 
    {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        if (m_isRealTime)
        {
            m_RealTime->setAmbient(glm::vec3(f[0], f[1], f[2]));
        }
        else
        {
            m_ambientColor = vec3(f[0], f[1], f[2]);
        }
    }

    else if (c == "brdf")
    {
        // syntax: brdf  Kd:r g b   Ks:r g b  roughness Kt(optional):r g b  indexOfRefraction(optional) 
        // First rgb is Diffuse reflection, second is specular reflection.
        // Creates a Material instance to be picked up by successive shapes
        if (m_isRealTime)
        {
            m_RealTime->brdf(glm::vec3(f[0], f[1], f[2]), glm::vec3(f[3], f[4], f[5]), f[6]);
        }
        else
        {
            m_currentMaterial.Kd = vec3(f[0], f[1], f[2]);
            m_currentMaterial.Ks = vec3(f[3], f[4], f[5]);
            m_currentMaterial.roughness = f[6];
            m_currentMaterial.isLight = false;

            if (f.size() == 11)
            {
                m_currentMaterial.Kt = vec3(f[7], f[8], f[9]);
                m_currentMaterial.indexOfRefraction = f[10];
            }
            else
            {
                m_currentMaterial.Kt = vec3(0.0, 0.0, 0.0);
                m_currentMaterial.indexOfRefraction = 0.0;
            }
        }
    }

    else if (c == "light") 
    {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        if (m_isRealTime)
        {
            m_RealTime->light(glm::vec3(f[0], f[1], f[2]));
        }
        else
        {
            m_currentMaterial.Kd = vec3(f[0], f[1], f[2]);
            m_currentMaterial.Ks = vec3(f[3], f[4], f[5]);
            m_currentMaterial.roughness = f[6];
            m_currentMaterial.isLight = true;
            m_nextShapeIsLight = true;
        }
    }
   
    else if (c == "sphere")
    {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        if (m_isRealTime)
        {
            m_RealTime->sphere(glm::vec3(f[0], f[1], f[2]), f[3]);
        }
        else
        {
            PushBackShape(new Sphere(vec3(f[0], f[1], f[2]), f[3]));
        }
    }

    else if (c == "box")
    {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        if (m_isRealTime)
        {
            m_RealTime->box(glm::vec3(f[0], f[1], f[2]), glm::vec3(f[3], f[4], f[5]));
        }
        else
        {
            PushBackShape(new Box(vec3(f[0], f[1], f[2]), vec3(f[3], f[4], f[5])));
        }
    }

    else if (c == "cylinder")
    {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        if (m_isRealTime)
        {
            m_RealTime->cylinder(glm::vec3(f[0], f[1], f[2]), glm::vec3(f[3], f[4], f[5]), f[6]);
        }
        else
        {
            PushBackShape(new Cylinder(vec3(f[0], f[1], f[2]), vec3(f[3], f[4], f[5]), f[6]));
        }
    }

    else if (c == "triangle")
    {
        // syntax: triangle v0x v0y v0z   v1x v1y v1z   v2x v2y v2z
        // Creates a triangle defined by three vertices
        if (m_isRealTime)
        {

        }
        else
        {
            PushBackShape(new Triangle(vec3(f[0], f[1], f[2]), vec3(f[3], f[4], f[5]), vec3(f[6], f[7], f[8])));
        }
    }

    else if (c == "mesh")
    {
        // syntax: mesh  qw qx qy qz   s  tx ty tz  filename
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
      ReadAssimpFile(f, strings[0]);
    }

    else if (c == "texture")
    {
        // syntax: texture  filename
        // Reads a texture, and applys it to the latest Material as a diffuse texture
        if (m_isRealTime)
        {
            m_RealTime->setTexture(strings[0]);
        }
    }

    // Used to switch antialiasing modes
    else if (c == "antialiasing")
    {
        switch (int(f[0]))
        {
        case 2:
            m_Antialiasing = Jittering;
            m_AntialiasingN = int(f[1]);
            break;
        case 1:
            m_Antialiasing = On;
            m_AntialiasingN = int(f[1]);
            break;
        case 0:
        default:
            m_Antialiasing = Off;
        }
    }

    // Used to be able to switch between arbitrary paths without re-compiling while debugging
    else if (c == "arbitraryFlag")
    {
        m_arbitraryFlag = int(f[0]);
    }

    // Used to specify a single pixel to render (for debugging)
    else if (c == "singlePixel")
    {
        m_singlePixel = true;
        m_pixelX = int(f[0]);
        m_pixelY = int(f[1]);
    }

    else if (c == "quat")
    {
        // syntax:  quat   angle axis   angle axis   angle axis ...
        // A list of angle axis rotations is accumulated into a
        // quaternion and printed.  This is useful for building the
        // quaternions used in several places in the scene file.
        //
        // For example, to compute a 90 degree rotation around X
        // followed by a 45 degree rotation around Z: 
        //   quat 90 1 45 3
        // prints 
        //   90 X --> (0.707 0.707 0.000 0.000) --> (0.707 0.707 0.000 0.000) 
        //   45 Z --> (0.924 0.000 0.000 0.383) --> (0.653 0.653 0.271 0.271)
        quat q, accum(1,0,0,0);
        for (int c=0;  c<f.size();  c+=2) {
            float w = cos(f[c]*PI/180.0/2.0); // Cos and sin of angle/2 (as required by quaternions)
            float d = sin(f[c]*PI/180.0/2.0);
            int axis = int(f[c+1]);    // Axis 1, 2, 3 (for X, Y, Z)
            #define C(a) ((axis==a) ? 1.0 : 0.0)
            q = quat(w, d*C(1), d*C(2), d*C(3));
            accum = q * accum;          // Accumulate product of individual quaternions into accum;
            printf("%4g %c --> (%5.3f %5.3f %5.3f %5.3f) --> (%5.3f %5.3f %5.3f %5.3f) \n",
                   f[c], " XYZ"[axis],   q.w, q.x, q.y, q.z,
                   accum.w, accum.x, accum.y, accum.z);
            // Various sanity checks of GLM's quaternions.  (I'm slightly distrustful of GLM.)
            //{ vec3 v = glm::rotate(accum, vec3(1,0,0)); printf("   (%5.3g %5.3g %5.3g)\n", v.x, v.y, v.z); }
            //{ vec3 v = glm::rotate(accum, vec3(0,1,0)); printf("   (%5.3g %5.3g %5.3g)\n", v.x, v.y, v.z); }
            //{ vec3 v = glm::rotate(accum, vec3(0,0,1)); printf("   (%5.3g %5.3g %5.3g)\n", v.x, v.y, v.z); }
              
        }
    }
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::PushBackShape(Shape* s)
{
    s->m_material = m_currentMaterial;
    if (m_nextShapeIsLight)
    {
        m_Objects.push_back(s);
        m_Lights.push_back(s);
        m_nextShapeIsLight = false;
    }
    else
    {
        m_Objects.push_back(s);
    }
}

void Scene::BuildKdTree()
{
    m_kdBVH = Eigen::KdBVH<float, 3, Shape*>(m_Objects.begin(), m_Objects.end());
}

void Scene::TraceImage(vec3* image, const int pass)
{
    m_Timer.Start();
    
    BuildKdTree();

    if (!m_singlePixel)
    {
#ifdef OMP_PARALLEL 
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
#endif
        for (int y = 0; y < m_Height; y++)
        {

            fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < m_Width; x++)
            {
                if (m_Antialiasing == Off)
                {
                    Ray ray = m_Camera.CalculateRayAtPixel((double)x, (double)y);

                    image[y*m_Width + x] = GetColor(ray);                    
                }
                else if (m_Antialiasing == On)
                {
                    vec3 colorAccumulation = vec3(0.0, 0.0, 0.0);
                    double invAAGridNumber = 1.0 / m_AntialiasingN;
                    for (int i = 0; i < m_AntialiasingN; ++i)
                    {
                        for (int j = 0; j < m_AntialiasingN; ++j)
                        {
                            Ray ray = m_Camera.CalculateRayAtPixel((double)x + (double)i * invAAGridNumber + invAAGridNumber / 2.0, (double)y + (double)j * invAAGridNumber + invAAGridNumber / 2.0);

                            colorAccumulation += GetColor(ray);                           
                        }
                    }
                    image[y*m_Width + x] = colorAccumulation / glm::pow2((double)m_AntialiasingN);
                }
                else if (m_Antialiasing == Jittering)
                {
                    vec3 colorAccumulation = vec3(0.0, 0.0, 0.0);
                    double invAAGridNumber = 1.0 / m_AntialiasingN;
                    for (int i = 0; i < m_AntialiasingN; ++i)
                    {
                        for (int j = 0; j < m_AntialiasingN; ++j)
                        {
                            Ray ray = m_Camera.CalculateRayAtPixel((double)x + (double)i * invAAGridNumber + U01random(prng) * invAAGridNumber, (double)y + (double)j * invAAGridNumber + U01random(prng) * invAAGridNumber);

                            colorAccumulation += GetColor(ray);
                        }
                    }
                    image[y*m_Width + x] = colorAccumulation / glm::pow2((double)m_AntialiasingN);
                }
            }
        }
    }
    else
    {
        Ray ray = m_Camera.CalculateRayAtPixel((double)m_pixelX, (double)m_pixelY);

        image[m_pixelY*m_Width + m_pixelX] = GetColor(ray);
    }
    fprintf(stderr, "\n");

    // Get the end time
    LARGE_INTEGER end;


    printf("TraceImage elapsed time: %f\n", m_Timer.Stop());
}

vec3 Scene::GetColor(const Ray& ray) const
{
    Intersection closestIntersection;
    closestIntersection.t = FLT_MAX;

    CastRayInScene(ray, closestIntersection);

    if (closestIntersection.t < FLT_MAX - FLT_EPSILON)
    {
        // ambient
        vec3 ambient = m_ambientColor * closestIntersection.object->m_material.Kd;
        return /*ambient +*/ Lighting(m_Camera.eyePos, closestIntersection, 0);
    }
    else
    {
        // Black if there is no intersection with any objects
        return vec3(0.0, 0.0, 0.0);
    }
}

void Scene::CastRayInScene(const Ray& ray, Intersection& closestIntersection) const
{
    if (m_usingBVH)
    {
        Minimizer minimizer(ray);
        Eigen::BVMinimize(m_kdBVH, minimizer);
        closestIntersection = minimizer.m_minimumIntersection;
    }
    else
    {
        // Brute force, compare against all objects
        for (size_t i = 0; i < m_Objects.size(); ++i)
        {
            Intersection intersection;
            intersection.t = FLT_MAX;
            m_Objects[i]->Intersect(ray, intersection);
            if (intersection.t < closestIntersection.t && intersection.t > FLT_EPSILON)
            {
                closestIntersection = intersection;
            }
        }
    }
}

double Characteristic(double d)
{
    if (d > 0)
    {
        return 1.0;
    }
    else
    {
        return 0.0;
    }
}

double D(double mDotN, double roughness)
{
    /*return Characteristic(mDotN) * ((roughness + 2.0) / (2.0 * PI)) * pow(mDotN, roughness);
    
    double denom = glm::pow2(mDotN) * (glm::pow4(roughness) - 1.0) + 1.0f;
    return glm::pow4(roughness) / (PI * glm::pow2(denom));*/

    // Trowbridge-Reitz GGX
    if (mDotN > 0)
    {
        return glm::pow4(roughness) / (PI * glm::pow2(glm::pow2(mDotN) * (glm::pow4(roughness) - 1.0) + 1.0));
    }
    else
    {
        return 0;
    }
}

double tanTheta(double vDotN)
{
    return sqrt(1.0 - glm::pow2(vDotN)) / vDotN;
}

double G1(double nDotV, double roughness)
{
    /*double a = sqrt((roughness / 2.0) + 1.0) / tanTheta(glm::dot(v, N));
    if (a < 1.6)
    {
        return Characteristic(glm::dot(v, m) / glm::dot(v, N)) * (3.53 * a + 2.181 * glm::pow2(a)) / (1.0 + 2.276 * a + 2.577 * glm::pow2(a));
    }
    else
    {
        return 1.0;
    }*/

    // Beckmann
    double c = nDotV / (glm::pow2(roughness) * sqrt(1.0 - glm::pow2(nDotV)));
    if (c < 1.6)
    {
        return (3.535 * c + 2.181 * glm::pow2(c)) / (1.0 + 2.276 * c + 2.577 * glm::pow2(c));
    }
    else
    {
        return 1.0;
    }
}

double G(double nDotV, double nDotL, double roughness)
{
    return G1(nDotV, roughness) * G1(nDotL, roughness);
}

vec3 F(double vDotH, vec3 Ks)
{
    // Schlick Fresnel approximation
    return Ks + (vec3(1.0, 1.0, 1.0) - Ks) * glm::pow(1.0 - vDotH, 5.0);
}

double G1V(double nDotV, double k)
{
    return 1.0 / (nDotV * (1.0 - k) + k);
}

vec3 Diffuse(vec3 L, vec3 N, vec3 Kd)
{
    return glm::saturate(glm::dot(N, L)) * Kd / PI;
}

vec3 Specular(vec3 L, vec3 N, vec3 V, vec3 Ks, double roughness, bool isTransmissive = false)
{
    double nDotL = glm::dot(N, L);
    double nDotV = glm::saturate(glm::dot(N, V));
    if (nDotL < 0)
    {
        N = -N;
        //indexOfRefraction = 1.0 / indexOfRefraction;
        nDotL = glm::saturate(glm::dot(N, L));
    }

    double denominator = 4.0 * glm::abs(nDotL) * glm::abs(nDotV);
    if (denominator > FLT_EPSILON)
    {
        vec3 H = glm::normalize(L + V);
        double lDotH = glm::saturate(glm::dot(L, H));
        double nDotH = glm::dot(N, H);
        if (nDotH < 0)
        {
            nDotH = glm::dot(-N, H);
        }
        nDotH = glm::saturate(nDotH);
        double vDotH = glm::saturate(glm::dot(V, H));

        double d = D(nDotH, roughness);
        vec3 f = F(vDotH, Ks);
        if (isTransmissive)
        {
            f = vec3(1.0, 1.0, 1.0) - f;
        }
        double g = G(nDotV, nDotL, roughness);

        return glm::saturate(d*g*f / denominator);
    }
    else
    {
        return vec3(0.0, 0.0, 0.0);
    }
}

vec3 Scene::Lighting(const vec3 eyePos, const Intersection& intersection, int recursionLevel) const
{
    ++recursionLevel;
    vec3 out = m_ambientColor * intersection.object->Kd();
    if (recursionLevel < 6)
    {
        for (size_t i = 0; i < m_Lights.size(); ++i)
        {
            // Cast a ray towards the light to see if this position is in shadow
            Sphere* light = static_cast<Sphere*>(m_Lights[i]);
            vec3 L = glm::normalize(light->m_center - intersection.position);
            vec3 N = glm::normalize(intersection.normal);
            vec3 V = glm::normalize(eyePos - intersection.position);

            // Cast a ray towards the light
            Ray lightRay(intersection.position, L);
            Intersection lightIntersection;
            lightIntersection.t = FLT_MAX;
            CastRayInScene(lightRay, lightIntersection);
            // If the light can actually be seen
            if (lightIntersection.t < FLT_MAX - FLT_EPSILON && lightIntersection.object->IsLight())
            {
                // Calculate the standard BRDF
                // Diffuse
                out += Diffuse(L, N, intersection.object->Kd());
                out += Specular(L, N, V, intersection.object->Ks(), intersection.object->Roughness());                
            }

            // Calculate nDotV and index of refraction
            double indexOfRefraction = indexOfRefractionAir / intersection.object->IndexOfRefraction();
            double nDotV = glm::dot(N, V);
            if (nDotV < 0)
            {
                N = -N;
                indexOfRefraction = 1.0 / indexOfRefraction;
            }
            nDotV = glm::saturate(glm::dot(N, V));


            if (intersection.object->Kt().r + intersection.object->Kt().g + intersection.object->Kt().b > 0.0)
            {
                // Calculate light coming from the transmissive direction
                // Cast a ray in the transmissive direction
                //vec3 I = -V;
                //double nDotI = glm::dot(N, I);
                //double squareRoot = glm::sqrt(1.0 - glm::pow2(indexOfRefraction) * (1.0 - glm::pow2(nDotI)));
                //vec3 T = glm::normalize((indexOfRefraction * nDotI - squareRoot) * N - indexOfRefraction * I);
                double squareRoot = glm::sqrt(1.0 - glm::pow2(indexOfRefraction) * (1.0 - glm::pow2(nDotV)));
                vec3 T = glm::normalize((indexOfRefraction * nDotV - squareRoot) * N - indexOfRefraction * V);
                //vec3 T = glm::normalize(indexOfRefraction * -V + (indexOfRefraction * nDotV - squareRoot) * N);
                Ray transmissionRay(intersection.position, T);
                Intersection transmissionIntersection;
                transmissionIntersection.t = FLT_MAX;
                CastRayInScene(transmissionRay, transmissionIntersection);

                // If the ray hits an object
                if (transmissionIntersection.t < FLT_MAX - FLT_EPSILON)
                {
                    double e = glm::e<double>();
                    // Calculate light coming from the reflection direction by recursively calling the lighting function
                    vec3 beersLaw = glm::pow(vec3(e, e, e), transmissionIntersection.t * glm::log(intersection.object->Kt()));
                    vec3 transSpec = Specular(T, N, V, intersection.object->Ks(), intersection.object->Roughness(), true);
                    vec3 transSpec2 = Specular(T, N, V, intersection.object->Ks(), intersection.object->Roughness(), true);
                    vec3 transLight = Lighting(intersection.position, transmissionIntersection, recursionLevel);
                    vec3 transLight2 = Lighting(intersection.position, transmissionIntersection, recursionLevel);
                    double nDotT = glm::dot(N, T);
                    if (transSpec.x + transSpec.y + transSpec.z <= FLT_EPSILON)
                    {
                        //For debugging purposes
                        //out += vec3(0.0, 1.0, 0.0);
                    }
                    if (transLight.x + transLight.y + transLight.z < FLT_EPSILON)
                    {
                        //For debugging purposes
                        //out += vec3(0.0, 0.0, 1.0);
                    }
                    
                    {
                        out += beersLaw * /*transSpec */ transLight;
                    }
                }
                else
                {
                    //For debugging purposes
                    //out += vec3(1.0, 0.0, 0.0);
                }
            }
            else
            {
                // Cast a ray in the reflection direction
                vec3 R = glm::normalize(2.0 * nDotV * N - V);
                Ray reflectionRay(intersection.position, R);
                Intersection reflectionIntersection;
                reflectionIntersection.t = FLT_MAX;
                CastRayInScene(reflectionRay, reflectionIntersection);

                // If the ray hits an object
                if (reflectionIntersection.t < FLT_MAX - FLT_EPSILON)
                {
                    // Calculate light coming from the reflection direction by recursively calling the lighting function
                    double nDotR = glm::dot(N, R);
                    out += nDotR * Specular(R, N, V, intersection.object->Ks(), intersection.object->Roughness()) * Lighting(intersection.position, reflectionIntersection, recursionLevel);
                }
            }

        }
    }
    --recursionLevel;
    return glm::saturate(out);
}

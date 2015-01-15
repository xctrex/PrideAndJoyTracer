//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <string>
#include <fstream>
#include <vector>

//#include <Eigen/StdVector>
//#include <Eigen_unsupported/Eigen/BVH>

const double PI = 3.14159;

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    double abs(double v) { return v>0 ? v : -v; }
#endif

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 prng;
std::uniform_real_distribution<> U01random(0.0, 1.0);
// Call U01random(prng) to get a uniformly distributed random number in [0,1].

#include "raytrace.h"
#include "realtime.h"

Scene::Scene() { realtime = new Realtime(); }
void Scene::createMaterial() { realtime->createMaterial(); }
void Scene::setTexture(const std::string path) { realtime->setTexture(path); }
void Scene::setKd(const glm::vec3 c) { realtime->setKd(c); }
void Scene::setAlpha(const float a) { realtime->setAlpha(a); }
void Scene::triangleMesh(std::vector<float>* pnt,
                  std::vector<float>* nrm,
                  std::vector<float>* tex,
                  std::vector<float>* tan,
                  std::vector<unsigned int>* tris) { realtime->triangleMesh(pnt, nrm, tex, tan, tris); }

void Scene::Command(const std::string c, const std::vector<double> f, const std::vector<std::string> strings)
{
    if (c == "screen") {
        // syntax: screen width height
        realtime->setScreen(int(f[0]),int(f[1]));
        width = int(f[0]);
        height = int(f[1]); }

    else if (c == "camera") {
        // syntax: camera x y z   qw qx qy qz   ry
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        realtime->setCamera(glm::vec3(f[0],f[1],f[2]), glm::quat(f[3],f[4],f[5],f[6]), f[7]); }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        realtime->setAmbient(glm::vec3(f[0], f[1], f[2])); }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // First rgb is Diffuse reflection, second is specular reflection.
        // Creates a Material instance to be picked up by successive shapes
        realtime->brdf(glm::vec3(f[0], f[1], f[2]), glm::vec3(f[3], f[4], f[5]), f[6]); }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        realtime->light(glm::vec3(f[0], f[1], f[2])); }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        realtime->sphere(glm::vec3(f[0], f[1], f[2]), f[3]); }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        realtime->box(glm::vec3(f[0], f[1], f[2]), glm::vec3(f[3], f[4], f[5])); }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        realtime->cylinder(glm::vec3(f[0], f[1], f[2]), glm::vec3(f[3], f[4], f[5]), f[6]); }

    else if (c == "mesh") {
        // syntax: mesh  qw qx qy qz   s  tx ty tz  filename
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
      ReadAssimpFile(f, strings[0]);  }

    else if (c == "texture") {
        // syntax: texture  filename
        // Reads a texture, and applys it to the latest Material as a diffuse texture
        realtime->setTexture(strings[0]); }

    else if (c == "quat") {
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
        glm::quat q, accum(1,0,0,0);
        for (int c=0;  c<strings.size();  c+=2) {
            float w = cos(f[c]*PI/180.0/2.0); // Cos and sin of angle/2 (as required by quaternions)
            float d = sin(f[c]*PI/180.0/2.0);
            int axis = int(f[c+1]);    // Axis 1, 2, 3 (for X, Y, Z)
            #define C(a) ((axis==a) ? d : 0.0)
            q = glm::quat(w, C(1), C(2), C(3));
            accum = q * accum;          // Accumulate product of individual quaternions into accum;
            printf("%4g %c --> (%5.3f %5.3f %5.3f %5.3f) --> (%5.3f %5.3f %5.3f %5.3f) \n",
                   f[c], " XYZ"[axis],   q.w, q.x, q.y, q.z,
                   accum.w, accum.x, accum.y, accum.z); }
    }

    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(vec3* image, const int pass)
{
    realtime->run();                          // Remove this (realtime stuff)

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {

        fprintf(stderr, "Rendering %4d\r", y);
        for (int x=0;  x<width;  x++) {
            vec3 color;
            if ((x-width/2)*(x-width/2)+(y-height/2)*(y-height/2) < 100*100)
                color = vec3(U01random(prng), U01random(prng), U01random(prng));
            else if (abs(x-width/2)<4 || abs(y-height/2)<4)
                color = vec3(0.0, 0.0, 0.0);
            else 
                color = vec3(1.0, 1.0, 1.0);
            image[y*width + x] = color;
        }
    }
    fprintf(stderr, "\n");
}

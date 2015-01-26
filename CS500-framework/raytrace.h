///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

// The vec* types are vectors of DOUBLES, since most raytracing
// calculations should be done with doubles.
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
typedef glm::dvec4 vec4;  
typedef glm::dvec3 vec3;  
typedef glm::dvec2 vec2;
typedef glm::dquat quat;

////////////////////////////////////////////////////////////////////////////////
// Scene

class Realtime;

class Scene {
public:
    int width, height;
    Realtime* realtime;         // Remove this (realtime stuff)

    Scene();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::string c,
                 const std::vector<double> f,
                 const std::vector<std::string> strings);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::vector<double>& f, const std::string path);
    // As ReadAssimpFile parses the information from the model file,
    // it will call these methods:
    void createMaterial();
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
};

////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include <glload/gl_3_3_comp.h>
#include <glload/gl_load.hpp>
#include <GL/freeglut.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/ext.hpp>

#include <glimg/glimg.h>

////////////////////////////////////////////////////////////////////////
// Shader programming class
////////////////////////////////////////////////////////////////////////
class ShaderProgram
{
public:
    int program;
    
    void CreateProgram() { program = glCreateProgram(); }
    void Use() { glUseProgram(program); }
    void Unuse() { glUseProgram(0); }

    void CreateShader(const std::string fname, const int type)
    {
        // Read a file into a string
        std::ifstream f;
        f.open(fname, std::ios_base::binary); // Open
        f.seekg(0, std::ios_base::end);       // Position at end
        int length = f.tellg();               // to get the length

        char* src = new char [length+1];  // Create buffer of needed length
        f.seekg (0, std::ios_base::beg);      // Position at beginning
        f.read (src, length);             //   to read complete file
        f.close();                            // Close

        src[length] = char(0);            // Finish with a NULL

        // Create a shader, attach, send it the source, and compile it.
        int shader = glCreateShader(type);
        glAttachShader(program, shader);
        glShaderSource(shader, 1, &src, NULL);
        glCompileShader(shader);
        
        // Get the compilation status
        int status;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
        
        // If compilation status is not OK, get and print the log message.
        if (status != 1) {
            int length;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
            char* buffer = new char[length];
            glGetShaderInfoLog(shader, length, NULL, buffer);
            printf("Compile log(%s):\n%s\n", type==GL_VERTEX_SHADER?"Vertex":"Fragment", buffer);
            delete buffer;
        }
    };

    void LinkProgram()
    {
        // Link program and check the status
        glLinkProgram(program);
        int status;
        glGetProgramiv(program, GL_LINK_STATUS, &status);
    
        // If link failed, get and print log
        if (status != 1) {
            int length;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
            char* buffer = new char[length];
            glGetProgramInfoLog(program, length, NULL, buffer);
            printf("Link log:\n%s\n", buffer);
            delete buffer;
        }
    }
};

////////////////////////////////////////////////////////////////////////
// Object store a Mesh pointer and modeling transformations
// Meshes stored OpenGL Vertex Array Objects (VAO)
////////////////////////////////////////////////////////////////////////

class Mesh
{
 public:
    std::vector<float>* pnt;
    std::vector<float>* nrm;
    std::vector<float>* tex;
    std::vector<float>* tan;
    std::vector<unsigned int>* tris;
    unsigned int vao;
    int count;

    Mesh()
    {    
        pnt = new std::vector<float>;
        nrm = new std::vector<float>;
        tex = new std::vector<float>;
        tan = new std::vector<float>;
        tris = new std::vector<unsigned int>;
    }

    Mesh(std::vector<float>* _pnt,
         std::vector<float>* _nrm,
         std::vector<float>* _tex,
         std::vector<float>* _tan,
         std::vector<unsigned int>* _tris)
    {    
        pnt = _pnt;
        nrm = _nrm; 
        tex = _tex; 
        tan = _tan; 
        tris = _tris;
        MakeVAO();
    }

    void MakeVAO();

    void draw()
    {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, 3*count, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
};

class Material
{
 public:
    glm::vec3 Kd, Ks;
    float alpha;
    int texid;

    virtual bool isLight() { return false; printf("false\n"); }

    Material()  : Kd(glm::vec3(1.0, 0.5, 0.0)), Ks(glm::vec3(1,1,1)), alpha(1.0), texid(0) {}
    Material(const glm::vec3 d, const glm::vec3 s, const float a) : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    void SetTexture(const std::string path)
    {
        try {
            glimg::ImageSet* img = glimg::loaders::stb::LoadFromFile(path);
            texid = glimg::CreateTexture(img, 0); }
        catch (glimg::loaders::stb::UnableToLoadException e) {
            printf("\nRead error:\n  %s\n\n", e.what());
            texid = 0; }
        glBindTexture(GL_TEXTURE_2D, texid);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 100);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    virtual void apply(const unsigned int program)
    {
        int loc = glGetUniformLocation(program, "Kd");
        glUniform3fv(loc, 1, &Kd[0]);

        loc = glGetUniformLocation(program, "Ks");
        glUniform3fv(loc, 1, &Ks[0]);

        loc = glGetUniformLocation(program, "alpha");
        glUniform1f(loc, alpha);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texid);
        loc = glGetUniformLocation(program, "tex");
        glUniform1i(loc, 0);

        loc = glGetUniformLocation(program, "emitter");
        glUniform1i(loc, 0);
    }
};

class Light: public Material
{
public:

    virtual bool isLight() { return true; printf("TRUE\n"); }

    Light(const glm::vec3 e) : Material() { Kd = e; }

    virtual void apply(const unsigned int program)
    {
        glm::vec3 Z;

        int loc = glGetUniformLocation(program, "Kd");
        glUniform3fv(loc, 1, &Kd[0]);

        loc = glGetUniformLocation(program, "emitter");
        glUniform1i(loc, 1);

    }
};

class Obj
{
public:
    glm::mat4 modelTR;
    Mesh* mesh;
    Material* material;
    glm::vec3 min, max;

    Obj(Mesh* m, const glm::mat4& tr, Material* b,
        const glm::vec3& _min, const glm::vec3& _max)
        : mesh(m), modelTR(tr), material(b), min(_min), max(_max) {}

    glm::vec3 center() { return 0.5f*(max+min); }
};

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Realtime
{
public:
    bool nav;
    char motionkey;
    float speed;

    // Camera/viewing parameters
    glm::vec3 ambient;
    glm::vec3 eye;                   // Position of eye for viewing scene
    glm::quat orient;           // Represents rotation of -Z to view direction
    float ry;
    float front, back;
    float spin, tilt;
    float cDist;               // Distance from eye to center of scene
    //float lightSpin, lightTilt, lightDist;

    int mouseX, mouseY;
    bool shifted;
    bool leftDown;
    bool middleDown;
    bool rightDown;

    Mesh *sphMesh;
    Mesh *boxMesh;
    Mesh *cylMesh;

    ShaderProgram lighting;

    int width, height;
    void setScreen(const int _width, const int _height) { width=_width;  height=_height; }
    void setCamera(const glm::vec3 _eye, const glm::quat _o, const float _ry) { eye=_eye; orient=_o; ry=_ry; }
    void setAmbient(const glm::vec3 _a) { ambient = _a; }
    
    std::vector<Obj*> objs;
    std::vector<Obj*> lights;
    std::vector<Material*> materials;

    Material* currentMaterial() { return materials.size() ? materials[materials.size()-1] : NULL; }

    glm::quat ViewQuaternion() {
        return glm::conjugate(glm::angleAxis(tilt-90.0f, 1.0f, 0.0f, 0.0f)
                              *glm::conjugate(orient)
                              *glm::angleAxis(spin, 0.0f, 0.0f, 1.0f));
    }

    glm::vec3 ViewDirection() {
        return glm::toMat3(ViewQuaternion())*glm::vec3(0.0f, 0.0f, -1.0f);
    }

    void DrawScene();
    void ReshapeWindow(int w, int h);
    void KeyboardUp(unsigned char key, int x, int y);
    void KeyboardDown(unsigned char key, int x, int y);
    void MouseButton(int button, int state, int x, int y);
    void MouseMotion(int x, int y);
    
    void sphere(const glm::vec3 center, const float r);
    void box(const glm::vec3 base, const glm::vec3 diag);
    void cylinder(const glm::vec3 base, const glm::vec3 axis, const float radius);

    void createMaterial() { materials.push_back(new Material(*currentMaterial())); }
    void setKd(const glm::vec3 c) { currentMaterial()->Kd = c; printf("setKd: %g %g %g\n", c[0], c[1], c[2]); }
    void setKs(const glm::vec3 c) { currentMaterial()->Ks = c; }
    void setAlpha(const float a) { currentMaterial()->alpha = a; }
    void setTexture(const std::string path) { currentMaterial()->SetTexture(path); }
    void triangleMesh(std::vector<float>* pnt,
                      std::vector<float>* nrm,
                      std::vector<float>* tex,
                      std::vector<float>* tan,
                      std::vector<unsigned int>* tris) {
        Mesh* m = new Mesh(pnt, nrm, tex, tan, tris);
        glm::vec3 min(0.0,0.0,0.0); // BOGUS
        glm::vec3 max(0.0,0.0,0.0); // BOGUS
        Obj* obj = new Obj(m, glm::mat4(1.0), currentMaterial(), min, max);
        objs.push_back(obj);
        if (currentMaterial()->isLight())
            lights.push_back(obj);
    }

    void brdf(const glm::vec3 Kd, const glm::vec3 Ks, const float alpha)  {
        materials.push_back(new Material(Kd, Ks, alpha));
    }

    void light(const glm::vec3 e)  {
        materials.push_back(new Light(e));
    }

    Realtime();
    void run();
};



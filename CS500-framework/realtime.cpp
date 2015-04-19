////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include <string>
#include <fstream>
#include <vector>

#include <glload/gl_3_3_comp.h>
#include <glload/gl_load.hpp>
#include <GL/freeglut.h>

#include "raytrace.h"
#include "realtime.h"

const float PI = 3.14159f;
const float rad = PI/180.0f;    // Convert degrees to radians

#define PUSH4(v, a,b,c,d) { v->push_back(a); v->push_back(b); v->push_back(c); v->push_back(d); }
#define PUSH3(v, a,b,c) { v->push_back(a); v->push_back(b); v->push_back(c); }
#define PUSH2(v, a,b) { v->push_back(a); v->push_back(b); }


// Stupid C++ needs callbacks to be static functions.
static Realtime* globalRealtime = NULL;
void CBDrawScene()  { globalRealtime->DrawScene(); } 
void CBReshapeWindow(int w, int h)  { globalRealtime->ReshapeWindow(w,h); }
void CBKeyboardDown(unsigned char key, int x, int y)  { globalRealtime->KeyboardDown(key, x, y); }
void CBKeyboardUp(unsigned char key, int x, int y)  { globalRealtime->KeyboardUp(key, x, y); } 
void CBMouseButton(int button, int state, int x, int y)  { globalRealtime->MouseButton(button, state, x, y); } 
void CBMouseMotion(int x, int y)  { globalRealtime->MouseMotion(x,y); }
void CBAnimate(int value)
{
    glutTimerFunc(30, CBAnimate, 1);
    // atime = 360.0*glutGet(GLUT_ELAPSED_TIME)/12000;
    glutPostRedisplay();
}

Mesh* SphMesh()
{
    Mesh* mesh = new Mesh();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n*2;  i++) {
        float s = i*2.0f*PI/float(n*2);
        for (unsigned int j=0;  j<=n;  j++) {
            float t = j*PI/float(n);
            float x = cos(s)*sin(t);
            float y = sin(s)*sin(t);
            float z = cos(t);
            PUSH4(mesh->pnt, x,y,z,1.0f);
            PUSH3(mesh->nrm, x,y,z);
            PUSH2(mesh->tex, s/(2*PI), t/PI);
            PUSH3(mesh->tan, -sin(s), cos(s), 0.0);
            if (i>0 && j>0) {
              PUSH3(mesh->tris, (i-1)*(n+1) + (j-1), (i-1)*(n+1) + (j  ), (i  )*(n+1) + (j  ));
              PUSH3(mesh->tris, (i-1)*(n+1) + (j-1), (i  )*(n+1) + (j  ), (i  )*(n+1) + (j-1));
            } } }
    mesh->MakeVAO();
    return mesh;
}

Mesh* BoxMesh()
{
    glm::mat4 face[6] = {
        glm::mat4(1.0),
        glm::rotate(180.0f, 1.0f, 0.0f, 0.0f),
        glm::rotate( 90.0f, 1.0f, 0.0f, 0.0f),
        glm::rotate(-90.0f, 1.0f, 0.0f, 0.0f),
        glm::rotate( 90.0f, 0.0f, 1.0f, 0.0f),
        glm::rotate(-90.0f, 0.0f, 1.0f, 0.0f)};
       
    glm::mat4 half = glm::translate(0.5f, 0.5f, 0.5f)*glm::scale(0.5f, 0.5f, 0.5f);
    Mesh* mesh = new Mesh();
    for (unsigned int f=0;  f<6;  f++) {
        glm::mat4 m4 = half*face[f];
        glm::mat3 m3 = glm::mat3(m4);
        for (unsigned int i=0;  i<2;  i++) {
            for (unsigned int j=0;  j<2;  j++) {
              glm::vec4 tpnt = m4*glm::vec4(float(2*i)-1.0f, float(2*j)-1.0f, 1.0f, 1.0f);
              glm::vec3 tnrm = m3*glm::vec3(0.0f, 0.0f, 1.0f);
              glm::vec3 ttan = m3*glm::vec3(1.0, 0.0, 0.0);
              PUSH4(mesh->pnt, tpnt[0], tpnt[1], tpnt[2], tpnt[3]);
              PUSH3(mesh->nrm, tnrm[0], tnrm[1], tnrm[2]);
              PUSH2(mesh->tex, float(i), float(j));
              PUSH3(mesh->tan, ttan[0], ttan[1], ttan[2]);
              PUSH3(mesh->tris, 4*f+0, 4*f+1, 4*f+3);
              PUSH3(mesh->tris, 4*f+0, 4*f+3, 4*f+2); } } }
    mesh->MakeVAO();
    return mesh;
}

Mesh* CylMesh()
{
    Mesh* mesh = new Mesh();
    unsigned int n = 20;
    float d = 2.0f*PI/float(n*2);
    for (unsigned int i=0;  i<=n;  i++) {
        float s = i*2.0f*PI/float(n);
        float x = cos(s);
        float y = sin(s);
        
        float z = 0.0f;
        PUSH4(mesh->pnt, x,y,z,1.0f);
        PUSH3(mesh->nrm, x,y,0.0f);
        PUSH2(mesh->tex, s/(2*PI), 0.0f);
        PUSH3(mesh->tan, -sin(s), cos(s), 0.0f);

        z = 1.0f;
        PUSH4(mesh->pnt, x,y,z,1.0f);
        PUSH3(mesh->nrm, x,y,0.0f);
        PUSH2(mesh->tex, s/(2*PI), 0.0f);
        PUSH3(mesh->tan, -sin(s), cos(s), 0.0f);

        if (i>0) {
          PUSH3(mesh->tris, (i-1)*2+1, (i-1)*2, (i  )*2);
          PUSH3(mesh->tris, (i-1)*2+1, (i  )*2, (i  )*2+1); } }
    mesh->MakeVAO();
    return mesh;
}

void Mesh::MakeVAO()
{
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint Pbuff;
    glGenBuffers(1, &Pbuff);
    glBindBuffer(GL_ARRAY_BUFFER, Pbuff);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*pnt->size(),
                 &(*pnt)[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (nrm->size() > 0) {
        GLuint Nbuff;
        glGenBuffers(1, &Nbuff);
        glBindBuffer(GL_ARRAY_BUFFER, Nbuff);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*nrm->size(),
                     &(*nrm)[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0); }

    if (tex->size() > 0) {
        GLuint Tbuff;
        glGenBuffers(1, &Tbuff);
        glBindBuffer(GL_ARRAY_BUFFER, Tbuff);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*tex->size(),
                     &(*tex)[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0); }

    if (tan->size() > 0) {
        GLuint Dbuff;
        glGenBuffers(1, &Dbuff);
        glBindBuffer(GL_ARRAY_BUFFER, Dbuff);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*tan->size(),
                     &(*tan)[0], GL_STATIC_DRAW);
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0); }

    GLuint Ibuff;
    glGenBuffers(1, &Ibuff);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Ibuff);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)*tris->size(),
                 &(*tris)[0], GL_STATIC_DRAW);

    count = (int)tris->size()/3;

    glBindVertexArray(0);

    delete pnt;                 // ?? Does this delete vector contents?
    delete nrm;
    delete tex;
    delete tan;
    delete tris;
}

// Constructor for Realtime.  Initializes OpenGL, GLUT,as well as the
// data elements of the class.
Realtime::Realtime()
{
    globalRealtime = this;
    // Initialize GLUT
    int argc = 0;
    char* argv;
    glutInit(&argc, &argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitContextVersion (3, 3);
    glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

    glutInitWindowSize(200, 200);
    glutCreateWindow("Class Framework");
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

    glutIgnoreKeyRepeat(true);
    glutDisplayFunc(&CBDrawScene);
    glutReshapeFunc(&CBReshapeWindow);
    glutKeyboardFunc(&CBKeyboardDown);
    glutKeyboardUpFunc(&CBKeyboardUp);
    glutMouseFunc(&CBMouseButton);
    glutMotionFunc(&CBMouseMotion);   
    glutTimerFunc(30, CBAnimate, 1);

    // Initialize OpenGL
    glload::LoadFunctions();

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    printf("GLSL Version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("Rendered by: %s\n", glGetString(GL_RENDERER));
    fflush(stdout);
    
    // Create the shader program
    lighting.CreateProgram();
    lighting.CreateShader("realtime.vert", GL_VERTEX_SHADER);
    lighting.CreateShader("realtime.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lighting.program, 0, "vertex");
    glBindAttribLocation(lighting.program, 1, "vertexNormal");
    glBindAttribLocation(lighting.program, 2, "vertexTexture");
    glBindAttribLocation(lighting.program, 3, "vertexTangent");
    lighting.LinkProgram();

    // Several generic meshes which can be transfofrmed to *any* sphere, box, or cylinder.
    sphMesh = SphMesh();
    boxMesh = BoxMesh();
    cylMesh = CylMesh();

    // Initialize various member attributes
    materials.push_back(new RealtimeMaterial());

    nav = false;
    spin = 0.0f;
    tilt = 90.0f;
    speed = 0.05f;
    front = 0.1f;
    back = 1000.0f;

    shifted = false;
    leftDown = false;
    middleDown = false;
    rightDown = false;
}

// This function enters the event loop.
void Realtime::run()
{
    cDist = glm::length(eye);
    glutReshapeWindow(width, height);
    glutMainLoop();
}
// Called when the scene needs to be redrawn.
void Realtime::DrawScene()
{
    glm::vec3 viewDir = ViewDirection();

    glm::vec2 dir2 = glm::normalize(glm::vec2(viewDir.x, viewDir.y));
    if (motionkey == 'w')
        eye += speed*glm::vec3(dir2.x, dir2.y, 0.0);
    if (motionkey == 's')
        eye -= speed*glm::vec3(dir2.x, dir2.y, 0.0);
    if (motionkey == 'd')
        eye += speed*glm::vec3(dir2.y, -dir2.x, 0.0);
    if (motionkey == 'a')
        eye -= speed*glm::vec3(dir2.y, -dir2.x, 0.0);
    if (motionkey == 'e')
        eye -= speed*glm::vec3(0.0f, 0.0f, -1.0f);
    if (motionkey == 'c')
        eye -= speed*glm::vec3(0.0f, 0.0f, 1.0f);

    int loc;

    glClearColor(0.25,0.25, 0.25, 1.0);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

    glm::mat4 WorldView;
    glm::mat4 R = glm::toMat4(glm::conjugate(ViewQuaternion()));
    WorldView = R*glm::translate(-eye);

    float rx = (ry*width)/height;
    glm::mat4 WorldProj = glm::frustum(-front*rx, front*rx, -front*ry, front*ry, front, back);

    lighting.Use();

    loc = glGetUniformLocation(lighting.program, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(WorldProj));

    loc = glGetUniformLocation(lighting.program, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(WorldView));

    loc = glGetUniformLocation(lighting.program, "ambient");
    glUniform3fv(loc, 1, &ambient[0]);

    loc = glGetUniformLocation(lighting.program, "eyePos");
    glUniform3fv(loc, 1, &eye[0]);

    glm::vec3 lightEmit[8];
    glm::vec3 lightPosn[8];
    int  lightNum = (int)lights.size();
    for (int i=0;  i<lightNum;  i++) {
        lightPosn[i] = lights[i]->center();
        lightEmit[i] = lights[i]->material->Kd; }

    loc = glGetUniformLocation(lighting.program, "lightNum");
    glUniform1i(loc, lightNum);

    loc = glGetUniformLocation(lighting.program, "lightPosn");
    glUniform3fv(loc, 8, &lightPosn[0][0]);

    loc = glGetUniformLocation(lighting.program, "lightEmit");
    glUniform3fv(loc, 8, &lightEmit[0][0]);
        
    for (unsigned int i=0;  i<objs.size();  i++) {
        Mesh* mesh = objs[i]->mesh;
        RealtimeMaterial* material = objs[i]->material;
        glm::mat4& modelTR = objs[i]->modelTR;
        glm::mat3 normalTR = glm::mat3(glm::inverse(modelTR)); 

        loc = glGetUniformLocation(lighting.program, "ModelTr");
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(modelTR));

        loc = glGetUniformLocation(lighting.program, "normalTR");
        glUniformMatrix3fv(loc, 1, GL_FALSE, glm::value_ptr(normalTR));

        material->apply(lighting.program);
        mesh->draw(); }

    lighting.Unuse();
    glutSwapBuffers();
}

// Called by GLUT when the window size is changed.
void Realtime::ReshapeWindow(int w, int h)
{
    if (w && h)
        glViewport(0, 0, w, h);
    width = w;
    height = h;

    // Force a redraw
    glutPostRedisplay();
}

// Called by GLUT for keyboard actions.
void Realtime::KeyboardDown(unsigned char key, int x, int y)
{
    switch(key) {
    case 9:
        nav = !nav;
        break;

    case 'v':
        printf("camera %g %g %g    %g %g %g %g    %g\n",
               eye[0], eye[1], eye[2],  orient.w, orient.x, orient.y, orient.z, ry);
        printf("screne %d %d\n", width, height);
        fflush(stdout);
        break;

    case 'w': case 's': case 'a': case 'd': case 'e': case 'c':
        motionkey = key;
        break;
        
    case 27:                    // Escape key
    case 'q':
        glutLeaveMainLoop();
        break;
    }
}

void Realtime::KeyboardUp(unsigned char key, int x, int y)
{
    motionkey = 0;
    fflush(stdout);
}

// Called by GLut when a mouse button changes state.
void Realtime::MouseButton(int button, int state, int x, int y)
{        
    // Record the position of the mouse click.
    mouseX = x;
    mouseY = y;

    // Test if the SHIFT keey was down for this mouse click
    shifted = glutGetModifiers() && GLUT_ACTIVE_SHIFT;

    // Ignore high order bits, set by some (stupid) GLUT implementation.
    button = button%8;

    // Figure out the mouse action, and handle accordingly
    // if (button == 3 && shifted) { // Scroll light in
    //     lightDist = pow(lightDist, 1.0f/1.02f); }

    // else if (button == 4 && shifted) { // Scroll light out
    //     lightDist = pow(lightDist, 1.02f); }

    if (button == GLUT_LEFT_BUTTON) {
        leftDown = (state == GLUT_DOWN); }

    else if (button == GLUT_MIDDLE_BUTTON) {
        middleDown = (state == GLUT_DOWN); }

    else if (button == GLUT_RIGHT_BUTTON) {
        rightDown = (state == GLUT_DOWN); }

    else if (button == 3) {
        glm::vec3 C = eye + cDist*ViewDirection();
        cDist = pow(cDist, 1.0f/1.02f); 
        eye = C - cDist*ViewDirection(); }

    else if (button == 4) {
        glm::vec3 C = eye + cDist*ViewDirection();
        cDist = pow(cDist, 1.02f);
        eye = C - cDist*ViewDirection(); }
    
    // else if (button == 3)
    //     ry = ry/1.02f;

    // else if (button == 4)
    //     ry = ry*1.02f;

    // Force a redraw
    glutPostRedisplay();
    fflush(stdout);
}

void Realtime::MouseMotion(int x, int y)
{
    // Calculate the change in the mouse position
    int dx = x-mouseX;
    int dy = y-mouseY;

    // if (leftDown && shifted) {  // Rotate light position
    //     lightSpin += dx/3.;
    //     lightTilt -= dy/3.; }

    if (leftDown) {        // Rotate light position
        if (nav) {
            spin += dx/2.0f;
            tilt += dy/2.0f; }
        else {
            glm::vec3 C = eye + cDist*ViewDirection();
            spin += dx/2.0f;
            tilt += dy/2.0f;
            eye = C - cDist*ViewDirection(); } }

    // if (middleDown && shifted) {
    //     lightDist = pow(lightDist, 1.0f-dy/200.0f);  }

    else if (middleDown) { }

    // if (rightDown && shifted) { }
    // else if (rightDown) {
    //     tr[0] += dx/40.0f;
    //     tr[1] -= dy/40.0f;  }

    // Record this position
    mouseX = x;
    mouseY = y;

    // Force a redraw
    glutPostRedisplay();
}


void Realtime::sphere(const glm::vec3 center, const float r)
{
    glm::mat4 m = glm::translate(center) * glm::scale(r,r,r);
    glm::vec3 rrr(r,r,r);
    Obj* obj = new Obj(sphMesh, m, currentRealtimeMaterial(), center - rrr, center + rrr);
    objs.push_back(obj);
    if (currentRealtimeMaterial()->isLight())
        lights.push_back(obj);
}

void Realtime::box(const glm::vec3 base, const glm::vec3 diag)
{
    glm::mat4 m = glm::translate(base) * glm::scale(diag[0],diag[1],diag[2]);
    Obj* obj = new Obj(boxMesh, m, currentRealtimeMaterial(), base, base + diag);
    objs.push_back(obj);
    if (currentRealtimeMaterial()->isLight())
        lights.push_back(obj);
}


void Realtime::cylinder(const glm::vec3 base, const glm::vec3 axis, const float radius)
{
    glm::vec3 Z(0.0f, 0.0f, 1.0f);
    glm::vec3 C = glm::normalize(axis);
    glm::vec3 B = glm::cross(C,Z);
    if (glm::length(B) <1e-8)
        B = glm::vec3(0,1,0);
    else
        B = glm::normalize(B);
    glm::vec3 A = glm::normalize(glm::cross(B,C));
    glm::mat4 R(A[0],A[1],A[2],0.0f,
                B[0],B[1],B[2],0.0f,
                C[0],C[1],C[2],0.0f,
                0.0f, 0.0f, 0.0f, 1.0);
    
    glm::mat4 m = glm::translate(base)*R*glm::scale(radius,radius,glm::length(axis));
    glm::vec3 rrr(radius,radius,radius);
    Obj* obj = new Obj(cylMesh, m, currentRealtimeMaterial(), base - rrr, base + axis + rrr);
    objs.push_back(obj);
    if (currentRealtimeMaterial()->isLight())
        lights.push_back(obj);
}

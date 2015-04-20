///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright © 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    #include <time.h> 
#endif

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "raytrace.h"

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene)
{
    std::ifstream input(inName.c_str());
    if (input.fail()) {
        fprintf(stderr, "File not found: %s\n", inName.c_str());
        fflush(stderr);
        exit(-1); }

    // For each line in file
    for (std::string line; getline(input, line); ) {
        // Parse the command
        std::stringstream lineStream(line);
        std::string command;
        lineStream >> command;
        if (command.size() == 0) continue; // Ignore blank lines
        if (line[0] == '#') continue;      // Ignore comment lines

        // Parse as a list of doubles
        std::vector<double> doubles;
        for (double f; lineStream >> f; ) {
            doubles.push_back(f); }

        // (Re)Parse as a list of strings
        std::stringstream lineStream2(line);
        lineStream2 >> command;
        std::vector<std::string> strings;
        for (std::string s; lineStream2 >> s; ) {
            strings.push_back(s); }
        // Remove strings already parsed as doubles
        strings.erase(strings.begin(), strings.begin()+doubles.size());

        // Pass the line's data to Command(...)
        scene->Command(command, doubles, strings);
    }

    input.close();
}

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, vec3* image, const int pass, const float scale=1.0)
{
    rgbe_header_info info;
    char errbuf[100] = {0};

    FILE* fp = _fsopen(outName.c_str(), "wb", _SH_DENYWR);

    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf, ARRAYSIZE(errbuf));
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    float* data = new float[width*height*3];
    float* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            vec3 pixel = image[y*width + x];
            *dp++ = (float)pixel[0]*(scale/pass);
            *dp++ = (float)pixel[1]*(scale/pass);
            *dp++ = (float)pixel[2]*(scale/pass); } }

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    delete data;
}

// Convert a double to a two byte integer, with high byte first.
void tobytes(unsigned char*& ptr, const double f)
{
    unsigned int i = unsigned(65535.0*f);
    if (i>65535) i=65535;
    if (i<0) i=0;
    *ptr++ = i/256;
    *ptr++ = i%256;
}

// Write the image as a PPM (16 bit) image.  
void WritePPMImage(const std::string outName, const int width, const int height, vec3* image, const int pass)
{
    FILE *f;
    unsigned char* bytes = new unsigned char[2*3*width*height];
    unsigned char* ptr = bytes;

    // Open file
    errno_t err = fopen_s(&f, outName.c_str(), "wb");

    // Write file header
    fprintf(f, "P6\n%d %d\n%d\n", width, height, 65535);
    ptr = bytes;
        
    // Loop through rows top to bottom, then pixels left to right,
    // then R,G,B,  converting each to a two byte quantity.
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            vec3& pixel = image[y*width + x];
            tobytes(ptr, pixel[0]);
            tobytes(ptr, pixel[1]/pass);
            tobytes(ptr, pixel[2]/pass); } }

    // Write bytes
    fwrite(bytes, 1, 2*3*width*height, f);
    fclose(f);
    free(bytes);
}


////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    Scene* scene = new Scene();

    // Read the command line argument
    std::string inName =  (argc > 1) ? argv[1] : "shapetest.scn";
    //std::string ppmName = inName;
    std::string hdrName = inName;

    //ppmName.replace(ppmName.size()-3, ppmName.size(), "ppm");
    hdrName.replace(hdrName.size()-3, hdrName.size(), "hdr");

    // Read the scene, calling scene.Command for each line.
    ReadScene(inName, scene);

    // Allocate and clear an image array
    vec3 *image =  new vec3[scene->m_Width * scene->m_Height];
    for (int y=0;  y<scene->m_Height;  y++)
        for (int x=0;  x<scene->m_Width;  x++)
            image[y*scene->m_Width + x] = vec3(0,0,0);

    // PathTrace the image
    scene->PathTraceImage(image, 1);

    // Write the image
    WriteHdrImage(hdrName, scene->m_Width, scene->m_Height, image, 1);
}

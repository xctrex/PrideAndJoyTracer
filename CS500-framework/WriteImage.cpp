#include "WriteImage.h"
// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, vec3* image, const int pass, const float scale /*= 1.0*/)
{
    rgbe_header_info info;
    char errbuf[100] = { 0 };

    FILE* fp = _fsopen(outName.c_str(), "wb", _SH_DENYWR);

    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf, ARRAYSIZE(errbuf));
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    float* data = new float[width*height * 3];
    float* dp = data;
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x<width; ++x) {
            vec3 pixel = image[y*width + x];
            *dp++ = (float)pixel[0] * (scale / pass);
            *dp++ = (float)pixel[1] * (scale / pass);
            *dp++ = (float)pixel[2] * (scale / pass);
        }
    }

    r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    delete data;
}
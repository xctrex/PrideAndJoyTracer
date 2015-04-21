#pragma once
#include "stdafx.h"
// Write the image as a HDR(RGBE) image.
void WriteHdrImage(const std::string outName, const int width, const int height, vec3* image, const int pass, const float scale = 1.0);
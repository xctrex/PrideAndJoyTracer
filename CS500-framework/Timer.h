#pragma once

#include <time.h>

class Timer
{
public:
    Timer();
    void Start();
    float Stop();
private:
    LARGE_INTEGER m_start;
    LARGE_INTEGER m_end;
    LARGE_INTEGER m_ticksPerSecond;
};

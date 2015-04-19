#include "stdafx.h"
#include "Timer.h"

Timer::Timer()
{
    QueryPerformanceFrequency(&m_ticksPerSecond);
}

void Timer::Start()
{
    QueryPerformanceCounter(&m_start);
}

float Timer::Stop()
{

    QueryPerformanceCounter(&m_end);
    // get the difference between the current time and the start time
    return (float)(m_end.QuadPart - m_start.QuadPart) / (float)(m_ticksPerSecond.QuadPart);
}
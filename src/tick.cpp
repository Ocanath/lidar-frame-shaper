#include "tick.h"

#if defined(_WIN32)
#include <windows.h>

uint64_t get_tick64()
{
    return static_cast<uint64_t>(GetTickCount64());
}

uint64_t get_microsecond64()
{
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return static_cast<uint64_t>(count.QuadPart * 1000000LL / freq.QuadPart);
}

#else
#include <time.h>

uint64_t get_tick64()
{
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000000ULL;
}

uint64_t get_microsecond64()
{
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

#endif

uint32_t get_tick32()
{
    return static_cast<uint32_t>(get_tick64());
}

uint32_t get_microsecond32()
{
    return static_cast<uint32_t>(get_microsecond64());
}

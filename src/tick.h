#pragma once

#include <stdint.h>

// Milliseconds since system boot (wraps at ~49 days for 32-bit)
uint32_t get_tick32();

// Milliseconds since system boot (no practical wrap)
uint64_t get_tick64();

// Microseconds since system boot (wraps at ~71 minutes for 32-bit)
uint32_t get_microsecond32();

// Microseconds since system boot (no practical wrap)
uint64_t get_microsecond64();

#pragma once

#include <stdint.h>

// Returns milliseconds since system boot (wraps at ~49 days for 32-bit)
uint32_t get_tick32();

// Returns milliseconds since system boot (no practical wrap)
uint64_t get_tick64();

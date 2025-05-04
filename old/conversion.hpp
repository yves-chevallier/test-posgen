#pragma once

#include <stdint.h>

#define DECIMAL2FLOAT (1.f / (1ULL << 32))
#define FLOAT2INC 4294967296.f // Ca vient d'ou cette constante bulgare?
#define INC2FLOAT (1.f / FLOAT2INC)

inline int64_t float2inc(float value) {
    return value * FLOAT2INC;
}  // TODO: limit check
inline float inc2float(int64_t value) { return value * INC2FLOAT; }
inline float incfrac2float(int64_t value) {
    return ((uint64_t)value & 0xFFFFFFFF) * DECIMAL2FLOAT;
}

inline int64_t positionToIn(int32_t position) {
    float feedConstant = 0.00249999994f;
    float gearRatio = 1.0f;
    float SIPrefixPosition = 9.99999997e-07f;

    return (int64_t)(float2inc(position / feedConstant / gearRatio *
                         SIPrefixPosition));
    }

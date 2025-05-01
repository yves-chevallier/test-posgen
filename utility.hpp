#pragma once



template <typename T>
inline T min(T a, T b) {
    return a < b ? a : b;
}

template <typename T>
inline T max(T a, T b) {
    return a > b ? a : b;
}

template <typename T>
inline T clamp(T value, T minv, T maxv) {
    return max<T>(min<T>(value, maxv), minv);
}

template <typename T>
inline T clamp(T value, T limit) {
    return max<T>(min<T>(value, limit), -limit);
}

#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

#pragma once

#include <cstdint>
#include <cmath>

class PositionGeneratorDouble {
public:
    void init(double dt);
    void move(int64_t targetPosQ32_32, double velocity, double acceleration, double deceleration);
    void moveAbsolute(int64_t absoluteTarget, int64_t currentPosition);
    int64_t update(); // returns delta Q32.32
    bool targetReached() const;
    void reset();
    void stop();

private:
    static constexpr double scale = 4294967296.0; // 2^32
    double dt = 0.0;

    // Motion profile
    double v_max = 0.0;
    double a_max = 0.0;
    double d_max = 0.0;

    // Internal state
    double velocity = 0.0;
    int64_t remaining = 0; // in Q32.32
    int sign = 1;
    bool finished = true;
};

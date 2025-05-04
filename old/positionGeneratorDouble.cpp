#include "positionGeneratorDouble.hpp"

void PositionGeneratorDouble::init(double dt) {
    this->dt = dt;
}

void PositionGeneratorDouble::move(int64_t targetPosQ32_32, double velocity, double acceleration, double deceleration) {
    v_max = velocity;
    a_max = acceleration;
    d_max = deceleration;

    remaining = targetPosQ32_32;
    sign = (remaining < 0) ? -1 : 1;
    remaining = std::abs(remaining);
    this->velocity = 0.0;
    finished = false;
}

void PositionGeneratorDouble::moveAbsolute(int64_t targetQ32_32, int64_t currentQ32_32) {
    int64_t delta = targetQ32_32 - currentQ32_32;
    move(delta, v_max, a_max, d_max);
}

int64_t PositionGeneratorDouble::update() {
    if (finished)
        return 0;

    double pos_remaining = static_cast<double>(remaining) / scale;

    // Compute braking distance needed to stop
    double stopping_distance = (velocity * velocity) / (2.0 * d_max);

    if (pos_remaining <= stopping_distance) {
        // Decelerate
        velocity = std::max(0.0, velocity - d_max * dt);
    } else {
        // Accelerate up to max velocity
        velocity = std::min(v_max, velocity + a_max * dt);
    }

    double step = velocity * dt;
    double stepQ32 = step * scale;

    if (stepQ32 >= remaining) {
        // Final step
        stepQ32 = remaining;
        finished = true;
    }

    remaining -= static_cast<int64_t>(stepQ32 + 0.5); // rounding
    return static_cast<int64_t>(stepQ32 + 0.5) * sign;
}

bool PositionGeneratorDouble::targetReached() const {
    return finished;
}

void PositionGeneratorDouble::reset() {
    remaining = 0;
    velocity = 0;
    finished = true;
    sign = 1;
}

void PositionGeneratorDouble::stop() {
    move(0, v_max, a_max, d_max);
}

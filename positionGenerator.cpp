#include "positionGenerator.hpp"
#include "utility.hpp"
#include "conversion.hpp"

#include <cmath>

#ifndef INT64_MAX
#    define INT64_MAX   0x7fffffffffffffffLL
#endif

PositionGenerator::PositionGenerator() :
      remainingPost(0),
      remaining(0),
      velocity(0),
      direction(0),
      sign(1) {
    context.velocity = 0;
    context.acceleration = 0;
    context.deceleration = 0;
      }

void PositionGenerator::move(int64_t pos, float velocity, float acceleration,
                             float deceleration) {
    context.velocity = velocity;
    context.acceleration = acceleration;
    context.deceleration = deceleration;
    move(pos);
}

void PositionGenerator::move(int64_t pos) {
    finished = false;
    profileVelNorm =
        clamp<int64_t>(float2inc(context.velocity * dt), 1, INT64_MAX);
    profileAccNorm =
        clamp<int64_t>(float2inc(context.acceleration * dt * dt), 1, INT64_MAX);
    profileDecNorm =
        clamp<int64_t>(float2inc(context.deceleration * dt * dt), 1, INT64_MAX);
    int64_t denom = 2 * profileDecNorm;
    int64_t stopDist =
        (velocity * (velocity + profileDecNorm) + denom - 1) / denom;
    if (pos * direction < stopDist) {
        remainingPost = stopDist - velocity - pos * direction;
        pos = stopDist - velocity;
    } else {
        remainingPost = 0;
        sign = pos < 0 ? -1 : 1;
    }
    remaining = llabs(pos);
    if(remaining < positionToIn(1.f)){
        remaining = 0;
    }
}

void PositionGenerator::moveAbsolute(int64_t pos, int64_t posActual, float velocity,
                                     float acceleration, float deceleration) {
    context.velocity = velocity;
    context.acceleration = acceleration;
    context.deceleration = deceleration;
    moveAbsolute(pos, posActual);
}

void PositionGenerator::moveAbsolute(int64_t pos, int64_t posActual) {
    debugPosition = pos - posActual;
    move(pos - posActual, context.velocity, context.acceleration, context.deceleration);
}

int64_t PositionGenerator::update() {
    if (UNLIKELY(remaining <= 0)) { //TODO: correct acceleration jump when switch velocity
        velocity = direction = 0;
        finished = true;
        if (remainingPost) {
            remaining = remainingPost;
            remainingPost = 0;
            sign = -sign;
        }
        return 0;
    }
    tmp = velocity + profileAccNorm;
    r = remaining * 2 * profileDecNorm;
    if (velocity * (velocity + profileDecNorm) >= r) {
        sqr = profileDecNorm * profileDecNorm + 4 * r;
        res = sqrtf(sqr);
        velocity =
            min<int64_t>((sqr - res * res < 0) ? (res - profileDecNorm) / 2 - 1
                                               : (res - profileDecNorm) / 2,
                         remaining);
    } else if (velocity > profileVelNorm)
        velocity = max<int64_t>(velocity - profileDecNorm, profileVelNorm);
    else if (tmp * (tmp + profileDecNorm) < r)
        velocity = min<int64_t>(tmp, profileVelNorm);
    remaining -= velocity;
    direction = sign;
    return velocity * sign;
}

void PositionGenerator::init(float dt) { this->dt = dt; }

void PositionGenerator::reset() {
    remainingPost = 0;
    remaining = 0;
    velocity = 0;
    direction = 0;
    sign = 1;
    finished = true;
}

void PositionGenerator::stop() { move(0); }

bool PositionGenerator::targetReached() { return finished; }

void PositionGenerator::setProfileVelocity(float velocity) {
    context.velocity = velocity;
}

void PositionGenerator::setProfileAcceleration(float acceleration) {
    context.acceleration = acceleration;
}

void PositionGenerator::setProfileDeceleration(float deceleration) {
    context.deceleration = deceleration;
}

PositionGeneratorContext PositionGenerator::getContext() { return context; }

void PositionGenerator::setContext(PositionGeneratorContext context) {
    this->context = context;
}

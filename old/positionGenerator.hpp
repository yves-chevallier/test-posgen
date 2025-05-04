#pragma once

#include <stdint.h>

class Axis;

struct PositionGeneratorContext {
    float velocity;               // rev/s
    float acceleration;              // rev/s^2
    float deceleration;              // rev/s^2
};

class PositionGenerator {
   private:
    PositionGeneratorContext context;
    int64_t profileVelNorm;
    int64_t profileAccNorm;
    int64_t profileDecNorm;
    int64_t remainingPost;
    int64_t remaining;
    int64_t velocity;
    int direction;
    int sign;
    bool finished;
    float dt;
    float velocity2inc(float v);
    float acceleration2inc(float a);

    int64_t tmp;
    int64_t r;
    int64_t sqr;
    int64_t res;
    int64_t debugPosition;

   public:
    PositionGenerator();
    void move(int64_t pos, float velocity, float acceleration,
              float deceleration);
    void move(int64_t pos);
    void moveAbsolute(int64_t pos, int64_t posActual, float velocity, float acceleration,
                      float deceleration);
    void moveAbsolute(int64_t pos, int64_t posActual);
    int64_t update();
    void init(float dt);
    void reset();
    void stop();
    bool targetReached();
    void setProfileVelocity(float velocityMax);
    void setProfileAcceleration(float acceleration);
    void setProfileDeceleration(float deceleration);
    PositionGeneratorContext getContext();
    void setContext(PositionGeneratorContext context);
};

#pragma once

#include <cmath>
#include "utility.hpp"

struct ContextScurvePlanner {

    double velocityLimit;
    double acceleration;
    double deceleration;
    double jerk_time;
};

class SCurvePlanner {
    double dt;
    double sign;

    double x;
    double acc;
    double dec;
    double vmax;
    double alpha;

    double v;
    double v_old;
    double dx_old;
    double movement_sign;

    bool breaking;
    bool finished;
    bool jerk_done;

    ContextScurvePlanner ctx;

public:
    SCurvePlanner(double dt)
        : dt(dt), sign(1.0),
          x(0.0), acc(0.0), dec(0.0), vmax(0.0), alpha(0.0),
          v(0.0), v_old(0.0), dx_old(0.0), movement_sign(1.0),
          breaking(false), finished(true), jerk_done(false) {}

    void set_context(const ContextScurvePlanner& ctx) {
        this->ctx = ctx;
    }

    void apply_context() {
        acc = ctx.acceleration * dt;
        dec = ctx.deceleration * dt;
        vmax = ctx.velocityLimit;
        alpha = (ctx.jerk_time > 0.0) ? dt / ctx.jerk_time : 0.0;
    }

    void set_distance(double value) {
        movement_sign = (value >= 0.0) ? 1.0 : -1.0;
        x = fabs(value);
    }

    void stop(double deceleration = -1.0) {
        if (deceleration >= 0.0)
            dec = deceleration;
        breaking = true;
    }

    bool start() {
        if (!finished)
            return false;
        v = 0.0;
        finished = false;
        breaking = false;
        jerk_done = false;
        apply_context();
        return true;
    }

    bool isFinished() const {
        return finished && jerk_done;
    }

    bool step(double& velocity_current, double& acceleration_current) {
        if (UNLIKELY(finished)) {
            apply_jerk(0.0, velocity_current, acceleration_current);
            return jerk_done;
        }

        v_old = v;

        if (UNLIKELY(breaking && x > dec)) {
            v -= dec;
            if (v <= 0.0) {
                v = 0.0;
                finished = true;
            }
        }

        if (UNLIKELY(x <= dec)) {
            v = x;
            x = 0.0;
            finished = true;
        } else {
            double left = 2.0 * dec * x;
            double right = v * (v + dec);

            if (left <= right) {
                v = (sqrt(dec * dec + 4.0 * left) - dec) / 2.0;
            } else {
                if (v + acc > vmax) {
                    v = v > vmax ? v - dec : vmax;
                } else {
                    v += acc;
                }
            }

            x -= v;

            if (v_old < v) {
                double left2 = 2.0 * dec * x;
                double right2 = v * (v + dec);
                if (left2 <= right2) {
                    v = v_old;
                    x += acc;
                }
            }
        }

        apply_jerk(v, velocity_current, acceleration_current);

        return isFinished();
    }

private:
    void apply_jerk(double sample, double& velocity_current, double& acceleration_current) {
        double dx;
        if (alpha > 0.0) {
            dx = dx_old + alpha * (sample - dx_old);
            jerk_done = std::fabs(dx - dx_old) < 1e-6;
        } else {
            dx = sample;
            jerk_done = true;
        }
        velocity_current = dx * movement_sign;
        acceleration_current = (dx - dx_old) * movement_sign;
        dx_old = dx;
    }
};

class Distretizer {
    double err;
public:
    Distretizer() : err(0.0) {}

    int step(double sample) {
        int x = static_cast<int>(std::round(sample));
        err += sample - x;

        if (fabs(err) >= 1.0) {
            int ierr = static_cast<int>(std::round(err));
            x += ierr;
            err -= ierr;
        }

        return x;
    }
};

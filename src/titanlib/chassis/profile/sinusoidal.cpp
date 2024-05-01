#include "titanlib/chassis/profile/sinusoidal.hpp"

namespace titanlib {

SinusoidalProfile::SinusoidalProfile(float totalTime, float distance) {
    this->totalTime = totalTime;
    this->distance = distance;
}

float SinusoidalProfile::getPosition(float time) {
    float u = M_PI / (2 * totalTime);
    if (time >= totalTime) {
        return distance;
    } else {
        return distance * sinf(u * time);
    }
}

float SinusoidalProfile::getVelocity(float time) {
    float u = M_PI / (2 * totalTime);
    if (time >= totalTime) {
        return 0;
    } else {
        return distance * cosf(u * time) * u;
    }
}

}
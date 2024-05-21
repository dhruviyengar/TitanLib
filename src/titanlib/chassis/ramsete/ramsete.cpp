#include "titanlib/chassis/ramsete/ramsete.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {

Ramsete::Ramsete(float b, float zeta) {
    this->b = b;
    this->zeta = zeta;
}

std::pair<float, float> Ramsete::ramseteOutput(Point pos, Point target, float heading, float targetHeading, float v, float w) {
    float theta = heading * (M_PI / 180);
    float thetaError = angleError(heading, targetHeading) * (M_PI / 180);
    Point error = target - pos;
    float eX = (error.getX() * cos(theta)) + (error.getY() * sin(theta));
    float eY = (error.getX() * -sin(theta)) + (error.getY() * cos(theta));
    float k = 2 * zeta * sqrtf(w * w + b * v * v);
    float vOutput = v * cos(thetaError) + k * eX;
    float wOutput = w + k * thetaError + ((b * v * sin(thetaError) * eY) / thetaError);
    return std::make_pair(vOutput, wOutput);
}

}
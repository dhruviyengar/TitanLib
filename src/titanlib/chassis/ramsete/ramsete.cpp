#include "titanlib/chassis/ramsete/ramsete.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {

Ramsete::Ramsete(float b, float zeta) {
  this->b = b;
  this->zeta = zeta;
}

std::pair<float, float> Ramsete::ramseteOutput(Point pos, Point target,
                                               float heading,
                                               float targetHeading, float v,
                                               float w, float t) {
  float eTheta = angleError(heading, targetHeading) * (M_PI / 180.0);
  float theta = heading * (M_PI / 180.0);
  float bearingError = angleError(heading, pos.angle(target)) * (M_PI / 180.0);
  float eX = -sin(bearingError) * pos.distance(target);
  float eY = -cos(bearingError) * pos.distance(target);
  float k = 2.0 * zeta * sqrtf((w * w) + b * (v * v));
  float vOutput = v * cos(eTheta) + (k * eX);
  float wOutput = w + (k * eTheta) + (b * v * sinc(eTheta) * eY);
  printf("%f %f %f %f %f %f %f %f\n", eX, eY, eTheta, vOutput, wOutput, v, w, t);
  return std::make_pair(vOutput, wOutput);
}

} // namespace titanlib
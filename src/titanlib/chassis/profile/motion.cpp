#include "titanlib/chassis/profile/motion.hpp"
#include "titanlib/chassis/bezier/bezier.hpp"

// https://github.com/treboB/MathStuff/blob/main/cubicBezierMotionProfile.py
namespace titanlib {

MotionPlan::MotionPlan(CubicBezier bezier, float maxAccel, float maxDeAccel,
                       float maxVel) {
  this->bezier = bezier;
  this->maxAccel = maxAccel;
  this->maxDeAccel = maxDeAccel;
  this->maxVel = maxVel;
}

void MotionPlan::generate(float ceiling) {
  float t = 0;
  float v = 0;
  float totalLength = bezier.arcLength(0, 1, true);
  float distanceTravelled = 0;
  while (t <= ceiling) {
    float deltaDist = t == 0 ? 0 : bezier.arcLength(t - 0.01, t, false);
    float k = bezier.getCurvature(t);
    float vCurvature = sqrtf(0.8 * 9.81 * (1.0 / k));
    float vMaxAccel = sqrtf(v * v + 2 * maxAccel * deltaDist);
    float vMaxDeAccel =
        sqrtf(0.8 * maxDeAccel * (totalLength - distanceTravelled));
    v = fmin(fmin(fmin(maxVel, vCurvature), vMaxAccel), vMaxDeAccel);
    if (v != vMaxDeAccel)
      v = fmax(sqrtf(v * v - 2 * vMaxDeAccel * deltaDist), v);
    float w = v * -k;
    linearVelocities[t] = v;
    angularVelocities[t] = w;
    distanceTravelled += deltaDist;
    t += 0.01;
  }
  linearVelocities[0] = getLinearVelocity(0.02);
  angularVelocities[0] = getAngularVelocity(0.02);
}

float MotionPlan::getLinearVelocity(float t) {
  for (std::pair<float, float> pair : linearVelocities) {
    if (fabs(pair.first - t) <= 0.01) {
      return pair.second;
    }
  }
  return 0;
}

float MotionPlan::getAngularVelocity(float t) {
  for (std::pair<float, float> pair : angularVelocities) {
    if (fabs(pair.first - t) <= 0.01) {
      return pair.second;
    }
  }
  return 0;
}

CubicBezier MotionPlan::getCurve() { return bezier; }

} // namespace titanlib
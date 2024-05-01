#pragma once

#include <cmath>

namespace titanlib {

struct PIDSettings {
  PIDSettings(float kP, float kD, float settleTime, float settleError,
              float maxVelChange)
      : kP(kP), kD(kD), settleTime(settleTime), settleError(settleError),
        maxVelChange(maxVelChange) {}

  float kP;
  float kD;
  float settleTime;
  float settleError;
  float maxVelChange;
};

class PID {
public:
  PID(PIDSettings settings);
  float update(float error, float delay);
  float getVel(float error);
  void reset();
  bool isSettled();
  PIDSettings settings = PIDSettings(0, 0, 0, 0, 0);

private:
  float prevVel = 0;
  float settleCount = 0;
  float prevError = 0;
};

} // namespace titanlib
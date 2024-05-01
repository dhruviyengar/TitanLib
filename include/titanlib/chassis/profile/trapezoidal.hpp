#pragma once

#include "profile.hpp"

namespace titanlib {

class TrapezoidalProfile : public MotionProfile {

    public:
        TrapezoidalProfile(float maxAccel, float maxVel, float distance);
        float getPosition(float time) override;
        float getVelocity(float time) override;

    private:
        float maxAccel;
        float maxVel;
        float distance;
        float accelTime;
        float accelDist;
        float cruiseTime;
        float cruiseDist;

};

}
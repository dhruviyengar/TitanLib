#include "titanlib/chassis/pid/pid.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib
{

    PID::PID(PIDSettings settings)
    {
        this->settings = settings;
    };

    float PID::update(float error, float delay)
    {
        float porportional = error * settings.kP;
        float derivative = (error - prevError) * settings.kD;
        float power = porportional + derivative;
        float powerChange = power - prevVel;
        if (fabs(error) < settings.settleError)
        {
            settleCount += delay;
        }
        else
        {
            settleCount = 0;
        }
        powerChange = constrainValue(powerChange, -settings.maxVelChange, settings.maxVelChange);
        float vel = prevVel + powerChange;
        prevVel = vel;
        prevError = error;
        return vel;
    }

    float PID::getVel(float error) {
        return error * settings.kP;
    }

    void PID::reset() {
        settleCount = 0;
        prevError = 0;
        prevVel = 0;
    }

    bool PID::isSettled()
    {
        return settleCount >= settings.settleTime;
    }

}
#include "titanlib/chassis/chassis.hpp"
#include "titanlib/chassis/util/util.hpp"

namespace titanlib {
    
void Chassis::updateOdom() {
    float verticalTravel = 0;
    float horizontalTravel = 0;
    float deltaTheta = angleError(getHeading(), imu->get_heading()) * (M_PI / 180);
    float theta = imu->get_heading() * (M_PI / 180);
    if (vertWheel == nullptr) {
        float left = ((((leftMotors->at(1).get_position() - prevPosLeft) / 360) * wheelSize * M_PI) / ratio) / deltaTheta;
        float right = ((((rightMotors->at(1).get_position() - prevPosRight) / 360) * wheelSize * M_PI) / ratio) / deltaTheta;
        float radius = (left + right) / 2;
        verticalTravel = 2 * radius * sin(deltaTheta / 2);
    } else {
        float radius = vertWheel->getDistanceTravelled() / deltaTheta;
        verticalTravel = 2 * radius * sin(deltaTheta / 2);
    }

    if (horzWheel != nullptr) {
        float radius = horzWheel->getDistanceTravelled() / deltaTheta;
        horizontalTravel = 2 * radius * sin(deltaTheta / 2);
    }

    Point verticalVector(verticalTravel * sin(theta), verticalTravel * cos(theta));
    Point horizontalVector(horizontalTravel * sin(theta + M_PI / 4), horizontalTravel * cos(theta + M_PI / 4));

    Point origin = pos.get();

    pos.set(origin + verticalVector + horizontalVector);
    heading.set(imu->get_heading());
    
    prevPosLeft = leftMotors->at(1).get_position();
    prevPosRight = rightMotors->at(1).get_position();

}

}
#pragma once

#include "titanlib/chassis/point/point.hpp"
namespace titanlib {

class Ramsete {

    public:
        Ramsete(float b, float zeta);
        std::pair<float, float> ramseteOutput(Point pos, Point target, float heading, float targetHeading, float v, float w);


    private:
        float b;
        float zeta;

};

}
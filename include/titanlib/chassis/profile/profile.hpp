#pragma once

namespace titanlib {
    
    class MotionProfile {

        public:
            virtual ~MotionProfile() = 0;
            virtual float getPosition(float time) = 0;
            virtual float getVelocity(float time) = 0;
            virtual float getTotalTime() = 0;


    };

}
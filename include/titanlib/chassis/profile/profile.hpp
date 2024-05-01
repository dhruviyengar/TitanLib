#pragma once

namespace titanlib {
    
    class MotionProfile {

        public:
            MotionProfile();
            virtual float getPosition(float time);
            virtual float getVelocity(float time);


    };

}
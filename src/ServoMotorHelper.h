#pragma once
#include "arduino.h"
#include <math.h> 

class ServoMotorHelper{
public:
    byte min_speed_angle = 100; //angle with min speed forward
    byte max_speed_angle = 180; //angle with max speed forward
    byte min_speed_angle_r = 180; //angle with min speed backward
    byte max_speed_angle_r = 0; //angle with max speed backward

    ServoMotorHelper(byte min_speed_angle, byte max_speed_angle, byte min_speed_angle_r, byte max_speed_angle_r){
        this->min_speed_angle = min_speed_angle;
        this->max_speed_angle = max_speed_angle;
        this->min_speed_angle_r = min_speed_angle_r;
        this->max_speed_angle_r = max_speed_angle_r;
    }

    /// @brief convert speed (-100;100) to angle (0; 180)
    /// @param speed 
    /// @return 0 and 180 - max speed. 90 - stop
    byte get_angle(int speed){
        if (speed == 0){
            return 90;
        }
        if (speed > 0)
            return _get_angle(min_speed_angle, max_speed_angle, speed);
        else
            return _get_angle(min_speed_angle_r, max_speed_angle_r, -speed);
    }
private:
    byte _get_angle(int min_angle, int max_angle, int speed){

        speed = constrain(speed, 0, 100);
        float ratio = ((float)(max_angle - min_angle)) / 100;

        byte result = round(min_angle + (ratio * speed));

        // Serial.printf("_get_angle: %d, %d, %d, %f, %d\n", min_angle, max_angle, speed, ratio, result);

        return result;
    }
};
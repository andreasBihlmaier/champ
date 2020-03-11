#ifndef PWM_SERVO_PLUGIN_H
#define PWM_SERVO_PLUGIN_H

#include <Arduino.h>
#include <Servo.h>
#include <macros/macros.h>

namespace DigitalServo
{
    class Plugin
    {
        Servo servo_;
        float current_angle_;
        float min_angle_;
        float max_angle_;
        float min_actuator_offset_;
        float max_actuator_offset_;
        bool inverted_;
        int inverter_;
        public:
            unsigned int leg_id;
            Plugin(int hardware_pin, float min_angle, float max_angle, int min_actuator_offset, int max_actuator_offset, bool inverted):
            current_angle_(0),    
            min_angle_(min_angle),
            max_angle_(max_angle),
            min_actuator_offset_(min_actuator_offset),
            max_actuator_offset_(max_actuator_offset),
            inverted_(inverted),
            inverter_(1),
            leg_id(0)
            {
                delay(100);
            
                if(inverted_)
                    inverter_ *= -1;
                
                servo_.attach(hardware_pin);
            }

            void positionControl(float angle)
            {
                current_angle_ = angle;
                servo_.write(toActuatorAngle(angle * inverter_));
            }

            float getJointPosition()
            {
                return current_angle_;
            }

            int toActuatorAngle(float angle)
            {
                
                float actuator_angle = 0;

                if(max_angle_ == 0)
                {   
                    actuator_angle =  map(angle, min_angle_, max_angle_, 180, 0);
                }
                else
                {
                    actuator_angle =  map(angle, min_angle_, max_angle_, 0, 180);
                }

                int offset = 0;
                
                if(min_actuator_offset_ == max_actuator_offset_)
                {
                    offset = min_actuator_offset_;
                }
                else
                {
                    offset = map(actuator_angle, 0, 180, min_actuator_offset_, max_actuator_offset_);
                }

                actuator_angle =  round(actuator_angle) + offset;

                return actuator_angle;
            }
    };
}

#endif


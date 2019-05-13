#include<Arduino.h>
#include "Servo.h"

class Actuators
{
private:
    String description;

public:
    Actuators(/* args */);
    ~Actuators();

    void moveServo(Servo &servo, int angle);
    //void logData(SD card, Data data)
};

Actuators::Actuators(/* args */)
{
}

Actuators::~Actuators()
{
}


#include <Arduino.h>
#include "Servo.h"

//MG995 datasheet: https://www.electronicoscaldas.com/datasheet/MG995_Tower-Pro.pdf
#define SERVO_BASE 1440 //  90° for unknow reason, should be 1500
#define PULSE 2666.6667 //  elapsed micros for 1° turn at 6V supply (actual time taken)
#define DEG_PULSE 10    //  how many pulse in 1° (pulse to command 1° turn, not speed or time taken)
#define SPEED 3         //  speed in millis for 1° turn (SPEED = PULSE / 1000)

class Actuator
{
private:
    String description;
    Servo servo;
    unsigned long lastUpdate = 0;
    unsigned long currentTime = 0;
    unsigned long delta_t = 0;
    unsigned long pos = SERVO_BASE;
    uint16_t target = SERVO_BASE;
    uint16_t map_angle(float degree);

public:
    Actuator();
    ~Actuator();

    void setupServo(int pin, int min, int max);
    void moveServo(int pulse);
    const String logServoPos();
    //void logData(SD card, Data data)
};

Actuator::Actuator() {}

Actuator::~Actuator()
{
}

void Actuator::setupServo(int pin, int min = 600, int max = 2400)
{
    servo.attach(pin, min, max);
    lastUpdate = millis();
    pos = SERVO_BASE;
    moveServo(SERVO_BASE);
}

void Actuator::moveServo(int pid_pulse)
{
    currentTime = millis();
    if (currentTime <= lastUpdate + this->delta_t)
    {
        return;
    }
    //input should be normalized but double check anyway
    if (pid_pulse < 1200)
        pid_pulse = 1200;
    if (pid_pulse > 1800)
        pid_pulse = 1800;
    target = pid_pulse;
    if (target == pos)
        return;
    lastUpdate = millis();
    this->delta_t = SPEED * abs(target - pos);
    servo.writeMicroseconds(target);
    pos = target;
}

const String Actuator::logServoPos()
{
    return (String)pos;
}

uint16_t Actuator::map_angle(float degree)
{
    // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    //in_min = 0; in_max = 180; out_min=600; out_max=2400;
    float pulse = degree * 10.f + 600.f;
    return round(pulse);
}

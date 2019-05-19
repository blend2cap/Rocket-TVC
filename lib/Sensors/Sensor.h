#pragma once
#include <MPU6050_6Axis_MotionApps20.h>

#define OUTPUT_READABLE

class Sensor
{
protected:
    String id;

public:
    Sensor(String id = "Sensor_001") {}
    ~Sensor() {}

    String getID() { return this->id; }
    inline void setID(String ID) { this->id = ID; }
};

class Gyroscope : public Sensor
{

public:
    double yaw;
    double pitch;
    double roll;
    double yaw_offset;
    double pitch_offset;
    double roll_offset;
    MPU6050 mpu;

private:
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];
    volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

public:
    Gyroscope();
    ~Gyroscope();

    void setup(int interrupt_pin);
    void readAngle(bool startup = false);
};

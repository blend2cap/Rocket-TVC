#pragma once
#include <MPU6050_6Axis_MotionApps20.h>

#define OUTPUT_READABLE

// class Sensor
// {
// protected:
//     String id;

// public:
//     Sensor(String id = "Sensor_001") {}
//     ~Sensor() {}

//     String getID() { return this->id; }s
//     inline void setID(String ID) { this->id = ID; }
// };

class Gyroscope 
{

public:
    double yaw;
    double pitch;
    double roll;
    double yaw_offset;
    double pitch_offset;
    double roll_offset;
    VectorInt16 acceleration;
    VectorInt16 gyroscope;
    MPU6050 mpu;


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


bool* ptr;
void dmpDataReady(){
    *ptr = true;
}
Gyroscope::Gyroscope()
{
    this->yaw = 0;
    this->pitch = 0;
    this->roll = 0;
    this->yaw_offset = 0;
    this->pitch_offset = 0;
    this->roll_offset = 0;
}
void Gyroscope::setup(int interrupt_pin){
    mpu.initialize();
    pinMode(interrupt_pin, INPUT);
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(159);

    if(devStatus == 0){
        mpu.setDMPEnabled(true);
        ptr=&dmpReady;
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus= mpu.getIntStatus();
        dmpReady=true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        for(int i=0; i<300; i++){
            readAngle(true);
        }
    }
}


void Gyroscope::readAngle(bool startup)
{
    // if programming failed, don't try to do anything
    if (dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!this->mpuInterrupt && this->fifoCount < this->packetSize)
    {
        if (this->mpuInterrupt && this->fifoCount < this->packetSize)
        {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    this->mpuInterrupt = false;
    this->mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    this->fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((this->mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || this->fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        this->fifoCount = mpu.getFIFOCount();

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (this->mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        // wait for correct available data length, should be a VERY short wait
        while (this->fifoCount < this->packetSize)
            this->fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(this->fifoBuffer, this->packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        this->fifoCount -= this->packetSize;

        // Get Yaw, pitch_pid and roll_pid values
#ifdef OUTPUT_READABLE
        mpu.dmpGetQuaternion(&this->q, this->fifoBuffer);
        mpu.dmpGetGravity(&this->gravity, &this->q);
        mpu.dmpGetAccel(&acceleration, fifoBuffer); //not sure about fifoBuffer
        mpu.dmpGetGyro(&gyroscope, fifoBuffer);
        mpu.dmpGetYawPitchRoll(this->ypr, &this->q, &this->gravity);

        // this->ypr[0] = sga_filter(this->ypr[0], ptr);
        // this->ypr[1] = sga_filter(this->ypr[1], ptr);
        // this->ypr[2] = sga_filter(this->ypr[2], ptr);

        // Yaw, pitch_pid, roll_pid values - Radians to degrees
        if (!startup)
        {
            this->yaw = (this->ypr[0] * 180 / M_PI) + this->yaw_offset;
            this->pitch = (this->ypr[1] * 180 / M_PI) + this->pitch_offset;
            this->roll = (this->ypr[2] * 180 / M_PI) + this->roll_offset;
        }
        else
        {
            this->yaw_offset = -(this->ypr[0] * 180 / M_PI);
            this->pitch_offset = -(this->ypr[1] * 180 / M_PI);
            this->roll_offset = -(this->ypr[2] * 180 / M_PI);
        }

#endif
    }
}



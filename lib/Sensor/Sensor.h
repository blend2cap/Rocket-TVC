#pragma once
#include <MPU6050_6Axis_MotionApps20.h>
#include <vector>

#define OUTPUT_READABLE
#define deltat 0.001f
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError
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

private:
    VectorInt16 acceleration;  //acceleration from IMU un-filtered
    VectorInt16 angular_speed; //gyro rad/s from IMU un-filtered
    Quaternion SEq;            //filtered SEq quaternion
    MPU6050 mpu;

    bool dmpReady = false;              // set true if DMP init was successful
    uint8_t mpuIntStatus;               // holds actual interrupt status byte from MPU
    uint8_t devStatus;                  // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;                // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;                 // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];             // FIFO storage buffer
    Quaternion q_raw;                   // [w, x, y, z]         raw quaternion container
    VectorFloat gravity;                // [x, y, z]            gravity vector
    volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

public:
    Gyroscope();
    ~Gyroscope();

    void setup(int interrupt_pin);
    void readAngle(bool startup = false);
    Quaternion Gyroscope::MadwickFilterUpdate();
    String Gyroscope::log_Orientation();
};

bool *ptr;
void dmpDataReady()
{
    *ptr = true;
}
Gyroscope::Gyroscope()
{
}
void Gyroscope::setup(int interrupt_pin)
{
    mpu.initialize();
    pinMode(interrupt_pin, INPUT);
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(159);

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        ptr = &dmpReady;
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        for (int i = 0; i < 300; i++)
        {
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
        mpu.dmpGetQuaternion(&this->q_raw, this->fifoBuffer);
        mpu.dmpGetGravity(&this->gravity, &this->q_raw);
        mpu.dmpGetAccel(&acceleration, fifoBuffer); //not sure about fifoBuffer
        mpu.dmpGetGyro(&angular_speed, fifoBuffer);
        //mpu.dmpGetYawPitchRoll(this->ypr, &this->q_raw, &this->gravity);

        // this->ypr[0] = sga_filter(this->ypr[0], ptr);
        // this->ypr[1] = sga_filter(this->ypr[1], ptr);
        // this->ypr[2] = sga_filter(this->ypr[2], ptr);

        // Yaw, pitch_pid, roll_pid values - Radians to degrees
        if (!startup)
        {
            // this->yaw = (this->ypr[0] * 180 / M_PI) + this->yaw_offset;
            // this->pitch = (this->ypr[1] * 180 / M_PI) + this->pitch_offset;
            // this->roll = (this->ypr[2] * 180 / M_PI) + this->roll_offset;
        }
        else
        {
            // this->yaw_offset = -(this->ypr[0] * 180 / M_PI);
            // this->pitch_offset = -(this->ypr[1] * 180 / M_PI);
            // this->roll_offset = -(this->ypr[2] * 180 / M_PI);
        }

#endif
    }
}

Quaternion Gyroscope::MadwickFilterUpdate()
{
    //Local system variables
    Quaternion SEqDot_omega;                                  // quaternion derivative from gyroscope elements
    float f_1, f_2, f_3;                                      // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    Quaternion SEqHatDot;                                     // estimated direction of the gyroscope error

    // Axulirary variables to avoid reapeated calcualtions
    Quaternion half_SEq(0.5 * SEq.w,
                        0.5 * SEq.x,
                        0.5 * SEq.y,
                        0.5 * SEq.z);
    Quaternion two_SEq(2 * SEq.w,
                       2 * SEq.x,
                       2 * SEq.y,
                       2 * SEq.z);

    // Normalise the accelerometer measurement
    acceleration.normalize();
    angular_speed.normalize();

    // Compute the objective function and Jacobian
    f_1 = two_SEq.x * SEq.z - two_SEq.w * SEq.y - acceleration.x;
    f_2 = two_SEq.w * SEq.x + two_SEq.y * SEq.z - acceleration.y;
    f_3 = 1.0f - two_SEq.x * SEq.x - two_SEq.y * SEq.y - acceleration.z;
    J_11or24 = two_SEq.y;
    J_12or23 = 2.0f * SEq.z;
    J_13or22 = two_SEq.w;
    J_14or21 = two_SEq.x;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    // J_11 negated in matrix multiplication
    // J_12 negated in matrix multiplication
    // negated in matrix multiplication
    // negated in matrix multiplication

    // Compute the gradient (matrix multiplication)
    SEqHatDot.w = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot.x = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot.y = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot.z = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    SEqHatDot.normalize();

    // Compute the quaternion derivative measured by gyroscopes

    SEqDot_omega.w = -half_SEq.x * angular_speed.x - half_SEq.y * angular_speed.y - half_SEq.z * angular_speed.z;
    SEqDot_omega.x = half_SEq.w * angular_speed.x + half_SEq.y * angular_speed.z - half_SEq.z * angular_speed.y;
    SEqDot_omega.y = half_SEq.w * angular_speed.y - half_SEq.x * angular_speed.z + half_SEq.z * angular_speed.x;
    SEqDot_omega.z = half_SEq.w * angular_speed.z + half_SEq.x * angular_speed.y - half_SEq.y * angular_speed.x;
    // Compute then integrate the estimated quaternion derivative
    SEq.w += (SEqDot_omega.w - (beta * SEqHatDot.w)) * deltat;
    SEq.x += (SEqDot_omega.x - (beta * SEqHatDot.x)) * deltat;
    SEq.y += (SEqDot_omega.y - (beta * SEqHatDot.y)) * deltat;
    SEq.z += (SEqDot_omega.z - (beta * SEqHatDot.z)) * deltat;

    // Normalise quaternion
    SEq.normalize();
    return SEq;
}

String Gyroscope::log_Orientation()
{
    String l = (String)SEq.w + "," +
               (String)SEq.x + "," +
               (String)SEq.y + "," +
               (String)SEq.z + ",";
    return l;
}
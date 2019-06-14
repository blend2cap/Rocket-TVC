#pragma once

#include <MPU6050_6Axis_MotionApps20.h>
#include <vector>

#define DEBUG
#define OUTPUT_READABLE
#define deltat 0.001f
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError

#ifdef DEBUG
#define LOG(X) Serial.println((X));
#else
#define LOG(X)
#endif

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
    VectorFloat EulerOrientation;
    Quaternion SEq; //filtered SEq quaternion
    MPU6050 mpu;

    bool dmpReady = false;              // set true if DMP init was successful
    uint8_t mpuIntStatus;               // holds actual interrupt status byte from MPU
    uint8_t devStatus;                  // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;                // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;                 // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];             // FIFO storage buffer
    Quaternion q_raw;                   // [w, x, y, z]         raw quaternion container from readAngle() function
    Quaternion q_offset;                //offset rotation quaternion
    Quaternion q_orientation;           //orientation quaternion = q_raw x q_offset*
    VectorFloat gravity;                // [x, y, z]            gravity vector
    volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
    //calibration variables
    int buffersize = 1000;
    uint8_t accel_deadzone = 8;
    uint8_t gyro_deadzone = 1;
    VectorInt16 meanAccel;
    VectorInt16 meanGyro;
    VectorInt16 acceleration_offset;
    VectorInt16 gyro_offset;
    int state = 0;
    //calibration methods
    void meansensors();
    void calibration();
    void Calibrate();
    void HardCodeCalibration();
    void setQuaternionOffset();

public:
    Gyroscope();
    void setup(int interrupt_pin);
    void readAngle();

    Quaternion MadwickFilterUpdate();
    Quaternion getOrientantion();
    VectorFloat getEuler() { return EulerOrientation; }

    String log_Orientation(bool quat_only = true);
    String log_raw_quaternion();
    String log_offsets();
    String log_euler();
};

volatile bool *ptr;
void dmpDataReady()
{
    *ptr = true;
}
Gyroscope::Gyroscope()
{
}
void Gyroscope::setup(int interrupt_pin)
{
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    //Calibrate(); //calibration happens without DMP
    HardCodeCalibration();
    pinMode(interrupt_pin, INPUT);
    Serial.println(mpu.testConnection() ? "Mpu connection successfull" : "MPU connection failed");
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0)
    {
        // Serial.println("Enabling DMP");
        LOG("Enabling DMP");
        mpu.setDMPEnabled(true);
        LOG("Enabling interrupt detection");
        ptr = &mpuInterrupt;
        attachInterrupt(digitalPinToInterrupt(interrupt_pin), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        LOG(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        //read first 500 values to get offset quaternion
        setQuaternionOffset();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print((String)devStatus + ")");
    }
}

void Gyroscope::setQuaternionOffset()
{
    LOG("Start Calculate offset");
    long oldTime, deltaTime, currentTime = 0L;
    deltaTime = 0L;
    currentTime = millis();
    while (deltaTime <= 15000L) //15 seconds
    {
        oldTime = currentTime;
        currentTime = millis();
        deltaTime = deltaTime + currentTime - oldTime;
        readAngle();
        q_offset = q_raw.getConjugate();
        Serial.print('.');
    }
}

void Gyroscope::readAngle()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
        LOG("Dmp is not ready");
        return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        LOG("fifoCount: " + (String)fifoCount + "\t" + "packetsize: " + (String)packetSize);

        //delay(100);
        if (mpuInterrupt && fifoCount < packetSize)
        {
            // try to get out of the infinite loop
            LOG("trying to get out of the infinite loop");
            fifoCount = mpu.getFIFOCount();
            LOG("fifoCount" + (String)fifoCount);
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        LOG("FIFO Overflow!");
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get Yaw, pitch_pid and roll_pid values

        if (mpu.dmpGetQuaternion(&q_raw, fifoBuffer) == 0)
        {
            mpu.dmpGetAccel(&acceleration, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q_raw);
            mpu.dmpGetGyro(&angular_speed, fifoBuffer);
            float ypr[3];
            // mpu.dmpGetYawPitchRoll(ypr, &q_raw, &gravity);
            mpu.dmpGetEuler(ypr, &q_raw);
            EulerOrientation.x = ypr[0];
            EulerOrientation.y = ypr[1];
            EulerOrientation.z = ypr[2];
            //LOG("dmp read successfully");
        }
        else
            LOG("cannot read quaternion");
    }
}

Quaternion Gyroscope::getOrientantion()
{
    q_orientation = q_raw.getProduct(q_offset);
    return q_orientation;
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

String Gyroscope::log_Orientation(bool quat_only)
{
    String header = "Orientation Quaternion:\t";
    Quaternion q = getOrientantion();
    String l = (String)q.w + "," +
               (String)q.x + "," +
               (String)q.y + "," +
               (String)q.z;
    if (quat_only)
    {
        return l;
    }
    else
    {
        return header + l;
    }
}

String Gyroscope::log_raw_quaternion()
{
    String header = "Raw Quaternion:\t";
    String l = (String)q_raw.w + "," +
               (String)q_raw.x + "," +
               (String)q_raw.y + "," +
               (String)q_raw.z;
    return l;
}

String Gyroscope::log_offsets()
{
    String header = "Offsets:\t";
    String l = (String)acceleration_offset.x + "," +
               (String)acceleration_offset.y + "," +
               (String)acceleration_offset.z + "," +
               (String)gyro_offset.x + "," +
               (String)gyro_offset.y + "," +
               (String)gyro_offset.z;
    return header + l;
}

String Gyroscope::log_euler()
{
    VectorFloat EulerDeg = EulerOrientation;
    EulerDeg.x = EulerOrientation.x * 180 / M_PI;
    EulerDeg.y = EulerOrientation.y * 180 / M_PI;
    EulerDeg.z = EulerOrientation.z * 180 / M_PI;
    String header = "Euler: \t";
    String l = (String)EulerDeg.x + "," +
               (String)EulerDeg.y + "," +
               (String)EulerDeg.z;
    //return header + l;
    return l;
}

void Gyroscope::meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
    while (i < (buffersize + 101))
    {
        mpu.getMotion6(&acceleration.x, &acceleration.y, &acceleration.z, &angular_speed.x, &angular_speed.y, &angular_speed.z);
        if (i > 100 && i <= (buffersize + 100)) //Discard first 100 readings
        {
            buff_ax = buff_ax + acceleration.x;
            buff_ay = buff_ay + acceleration.y;
            buff_az = buff_az + acceleration.z;
            buff_gx = buff_gx + angular_speed.x;
            buff_gy = buff_gy + angular_speed.y;
            buff_gz = buff_gz + angular_speed.z;
        }
        if (i == (buffersize + 100))
        {
            meanAccel.x = buff_ax / buffersize;
            meanAccel.y = buff_ay / buffersize;
            meanAccel.z = buff_az / buffersize;
            meanGyro.x = buff_gx / buffersize;
            meanGyro.y = buff_gy / buffersize;
            meanGyro.z = buff_gz / buffersize;
        }
        i++;
        //comment delay because could make one loop iteration slower than 10ms thus causing FIFO overflow at 100hz
        //delay(2); //Needed so we don't get repeated measures
    }
}

void Gyroscope::calibration()
{
    acceleration_offset.x = -meanAccel.x / 8;
    acceleration_offset.y = -meanAccel.y / 8;
    acceleration_offset.z = -meanAccel.z / 8;

    gyro_offset.x = -meanGyro.x / 4;
    gyro_offset.y = -meanGyro.y / 4;
    gyro_offset.z = -meanGyro.z / 4;
    while (1)
    {
        int ready = 0;
        mpu.setXAccelOffset(acceleration_offset.x);
        mpu.setYAccelOffset(acceleration_offset.y);
        mpu.setZAccelOffset(acceleration_offset.z);

        mpu.setXGyroOffset(gyro_offset.x);
        mpu.setYGyroOffset(gyro_offset.y);
        mpu.setZGyroOffset(gyro_offset.z);
        meansensors();
        if (abs(meanAccel.x) <= accel_deadzone)
            ready++;
        else
            acceleration_offset.x = acceleration_offset.x - meanAccel.x / accel_deadzone;

        if (abs(meanAccel.y) <= accel_deadzone)
            ready++;
        else
            acceleration_offset.y = acceleration_offset.y - meanAccel.y / accel_deadzone;

        if (abs(16384 - meanAccel.z) <= accel_deadzone)
            ready++;
        else
            acceleration_offset.z = acceleration_offset.z + (16384 - meanAccel.z) / accel_deadzone;

        if (abs(meanGyro.x) <= gyro_deadzone)
            ready++;
        else
            gyro_offset.x = gyro_offset.x - meanGyro.x / (gyro_deadzone + 1);

        if (abs(meanGyro.y) <= gyro_deadzone)
            ready++;
        else
            gyro_offset.y = gyro_offset.y - meanGyro.y / (gyro_deadzone + 1);

        if (abs(meanGyro.z) <= gyro_deadzone)
            ready++;
        else
            gyro_offset.z = gyro_offset.z - meanGyro.z / (gyro_deadzone + 1);

        if (ready == 6)
            break;
        LOG("Ready= " + (String)ready);
    }
}

void Gyroscope::Calibrate()
{
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    LOG("Calibration state: " + (String)state);

    while (1)
    {
        if (state == 0)
        {
            LOG("Reading sensors for the first time");
            meansensors();
            state++;
            // delay(100);
        }
        if (state == 1)
        {
            LOG("Calculating offsets...");
            calibration();
            state++;
            // delay(100);
        }
        if (state == 2)
        {
            meansensors();
            LOG("Calibration completed");
            LOG(this->log_offsets());
            break;
        }
    }
}
void Gyroscope::HardCodeCalibration()
{
    mpu.setXAccelOffset(-770);
    mpu.setYAccelOffset(1700);
    mpu.setZAccelOffset(1330);
    mpu.setXGyroOffset(157);
    mpu.setYGyroOffset(13);
    mpu.setZGyroOffset(37);
}
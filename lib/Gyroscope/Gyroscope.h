#pragma once

#include <MPU6050_6Axis_MotionApps20.h>
#include <vector>

//#define DEBUG
//#define STR_LOG

#ifdef DEBUG
#define LOG(X) Serial.println((X));
#else
#define LOG(X)
#endif

class Gyroscope
{

private:
    VectorInt16 acceleration;  //acceleration from IMU un-filtered
    VectorInt16 angular_speed; //gyro rad/s from IMU un-filtered

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
    const uint16_t buffersize = 1000;
    const uint8_t accel_deadzone = 8;
    const uint8_t gyro_deadzone = 1;
    VectorInt16 meanAccel;
    VectorInt16 meanGyro;
    VectorInt16 acceleration_offset;
    VectorInt16 gyro_offset;
    uint8_t state = 0;
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

    Quaternion getOrientantion();
    VectorFloat getEuler();
    VectorInt16 getAcceleration();

#ifdef STR_LOG
    String log_Orientation(bool showHeader = false);
    String log_raw_quaternion();
    String log_offsets();
    String log_euler(bool showHeader = false);
    String log_acceleration(bool showHeader = false);
#endif
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
        LOG("DMP ready! Waiting for first interrupt...");
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        //let gyro stabilize for 15 seconds to self-calibrate
        setQuaternionOffset();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        //Serial.print((String)devStatus + ")");
    }
}

void Gyroscope::setQuaternionOffset()
{
    LOG("Start Calculate offset");
    long oldTime, deltaTime, currentTime = 0L;
    deltaTime = 0L;
    currentTime = millis();
    while (deltaTime <= 8000L) //8 seconds
    {
        oldTime = currentTime;
        currentTime = millis();
        deltaTime = deltaTime + currentTime - oldTime;
        readAngle();
        q_offset = q_raw.getConjugate();
        LOG('.');
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
        if (mpuInterrupt) //yes I'm paranoid
            break;
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

        // clear FIFO as suggested here https://arduino.stackexchange.com/questions/10308/how-to-clear-fifo-buffer-on-mpu6050
        mpu.resetFIFO();
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get Yaw, pitch_pid and roll_pid values

        if (mpu.dmpGetQuaternion(&q_raw, fifoBuffer) == 0)
        {
            mpu.dmpGetAccel(&acceleration, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q_raw);
            mpu.dmpGetGyro(&angular_speed, fifoBuffer);
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

VectorFloat Gyroscope::getEuler()
{
    VectorFloat eul;
    Quaternion quat = getOrientantion();
    // Quaternion quat = q_raw;
    double sqw = quat.w * quat.w;
    double sqx = quat.x * quat.x;
    double sqy = quat.y * quat.y;
    double sqz = quat.z * quat.z;
    double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    double test = quat.x * quat.y + quat.z * quat.w;
    if (test > 0.499 * unit)
    { // singularity at north pole
        eul.y = 2 * atan2(quat.x, quat.w);
        eul.z = M_PI_2;
        eul.x = 0;
        return eul;
    }
    if (test < -0.499 * unit)
    { // singularity at south pole
        eul.y = -2 * atan2(quat.x, quat.w);
        eul.z = -M_PI_2;
        eul.x = 0;
        return eul;
    }
    eul.y = atan2(2 * quat.y * quat.w - 2 * quat.x * quat.z, sqx - sqy - sqz + sqw);
    eul.z = asin(2 * test / unit);
    eul.x = atan2(2 * quat.x * quat.w - 2 * quat.y * quat.z, -sqx + sqy - sqz + sqw);
    return eul;
}

VectorInt16 Gyroscope::getAcceleration()
{
    return this->acceleration;
}

#ifdef STR_LOG

String Gyroscope::log_Orientation(bool showHeader)
{
    Quaternion q = getOrientantion();
    String l = (String)q.w + "," +
               (String)q.x + "," +
               (String)q.y + "," +
               (String)q.z + ",";
    if (showHeader)
    {
        String header = "Orientation Quaternion:\t";
        return header + l;
    }
    return l;
}

String Gyroscope::log_raw_quaternion()
{
    String header = "Raw Quaternion:\t";
    String l = (String)q_raw.w + "," +
               (String)q_raw.x + "," +
               (String)q_raw.y + "," +
               (String)q_raw.z + ",";
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
               (String)gyro_offset.z + ", ";
    return header + l;
}

String Gyroscope::log_euler(bool showHeader)
{
    VectorFloat EulerDeg = getEuler();
    EulerDeg.x = EulerDeg.x * 180 / M_PI;
    EulerDeg.y = EulerDeg.y * 180 / M_PI;
    EulerDeg.z = EulerDeg.z * 180 / M_PI;
    String l = (String)EulerDeg.x + "," +
               (String)EulerDeg.y + "," +
               (String)EulerDeg.z + ",";
    if (showHeader)
    {
        String header = "Euler: \t";
        return header + l;
    }
    return l;
}

String Gyroscope::log_acceleration(bool showHeader)
{
    String l = (String)acceleration.x + "," +
               (String)acceleration.y + "," +
               (String)acceleration.z + ",";
    if (showHeader)
    {
        String header = "Accelerations: \t";
        return header + l;
    }
    return l;
}
#endif
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
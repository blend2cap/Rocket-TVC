#include <Sensor.h>

Gyroscope::Gyroscope()
{
    this->yaw = 0;
    this->pitch = 0;
    this->roll = 0;
    this->yaw_offset = 0;
    this->pitch_offset = 0;
    this->roll_offset = 0;
}

Gyroscope::~Gyroscope()
{
}

bool *tempDmp;
void copydmp(bool *mpuInterrupt)
{ //perform before attachInterrupt(...)
    *tempDmp = mpuInterrupt;
}
inline void dmpDataReady()
{
    *tempDmp = true;
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

void Gyroscope::setup(int interrupt_pin)
{
    mpu.initialize();
    pinMode(interrupt_pin, INPUT);
    this->devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(159);
    mpu.setYGyroOffset(14);
    mpu.setZGyroOffset(36);
    mpu.setZAccelOffset(1353); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (this->devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        copydmp(&this->dmpReady);
        attachInterrupt(digitalPinToInterrupt(interrupt_pin), dmpDataReady, RISING);
        this->mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        this->dmpReady = true;

        // get expected DMP packet size for later comparison
        this->packetSize = mpu.dmpGetFIFOPacketSize();
    } //there was else here
    for (size_t i = 0; i < 300; i++)
    {
        readAngle(true);
    }
}

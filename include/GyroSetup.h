/*
gyroscope setup void
attachInterrupt needs a static member so we put this code here aside
*/
#ifndef GYROSETUP
#define GYROSETUP
#include "Arduino.h"

void setupAngleIntegration(AngleIntegration *angleIntegral, MPU6050 *mpu, int interrupt_pin){
	// join I2C bus (I2Cdev library doesn't do angleIntegral automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment angleIntegral line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(38400);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately

	// initialize device
	//Serial.println(F("Initializing I2C devices..."));
	mpu->initialize();
	pinMode(interrupt_pin, INPUT);
	angleIntegral->devStatus = mpu->dmpInitialize();
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu->setXGyroOffset(159);
	mpu->setYGyroOffset(14);
	mpu->setZGyroOffset(36);
	mpu->setZAccelOffset(1353); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (angleIntegral->devStatus == 0) {
		// turn on the DMP, now that it's ready
		// Serial.println(F("Enabling DMP..."));
		mpu->setDMPEnabled(true);
		attachInterrupt(digitalPinToInterrupt(interrupt_pin), angleIntegral->dmpDataReady, RISING);
		angleIntegral->mpuIntStatus = mpu->getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		//Serial.println(F("DMP ready! Waiting for first interrupt..."));
		angleIntegral->dmpReady = true;

		// get expected DMP packet size for later comparison
		angleIntegral->packetSize = mpu->dmpGetFIFOPacketSize();
	}
}
#endif

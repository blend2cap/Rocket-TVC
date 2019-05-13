//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"
/*
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
*/
#include "Variables.h"
#include "PID_v1.h"
#include <microsmooth.h>
#include <Sensor.h>
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
Servo y_servo;
Servo x_servo;
Variables vars;
Gyroscope gyro;
double yaw_servoPIDvalue;
double Setpoint = 0;
double kp = 1, kd = 0.25, ki = 0.05;
double yaw, pitch, roll;
uint16_t *ptr;
PID yawPID(&yaw, &yaw_servoPIDvalue, &Setpoint, kp, ki, kd, DIRECT);

#define OUTPUT_READABLE_YAWPITCHROLL
#define Y_SERVO_PIN 9
#define X_SERVO_PIN 8
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

void startUp(Servo &servo, String command)
{
  int servoReadAngle;
  int intcommand = command.toInt();
  Serial.println("Type command: ");
  switch (intcommand)
  {
  case 8:
    servoReadAngle = servo.read();
    servo.write(++servoReadAngle);
    break;
  case 9:
    servoReadAngle = servo.read();
    servo.write(--servoReadAngle);
    break;
  }
}

void dmpDataReady()
{
  vars.mpuInterrupt = true;
}
void readAngle()
{
  // if programming failed, don't try to do anything
  if (!vars.dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!vars.mpuInterrupt && vars.fifoCount < vars.packetSize)
  {
    if (vars.mpuInterrupt && vars.fifoCount < vars.packetSize)
    {
      // try to get out of the infinite loop
      vars.fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  vars.mpuInterrupt = false;
  vars.mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  vars.fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((vars.mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || vars.fifoCount >= 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    vars.fifoCount = mpu.getFIFOCount();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (vars.mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
  {
    // wait for correct available data length, should be a VERY short wait
    while (vars.fifoCount < vars.packetSize)
      vars.fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(vars.fifoBuffer, vars.packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    vars.fifoCount -= vars.packetSize;

    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&vars.q, vars.fifoBuffer);
    mpu.dmpGetGravity(&vars.gravity, &vars.q);
    mpu.dmpGetYawPitchRoll(vars.ypr, &vars.q, &vars.gravity);

    // vars.ypr[0] = sga_filter(vars.ypr[0], ptr);
    // vars.ypr[1] = sga_filter(vars.ypr[1], ptr);
    // vars.ypr[2] = sga_filter(vars.ypr[2], ptr);

    // Yaw, Pitch, Roll values - Radians to degrees
    vars.ypr[0] = (vars.ypr[0] * 180 / M_PI) + vars.angleOffsets[0];
    vars.ypr[1] = (vars.ypr[1] * 180 / M_PI) + vars.angleOffsets[1];
    vars.ypr[2] = (vars.ypr[2] * 180 / M_PI) + vars.angleOffsets[2];
    vars.setDoubleAngles(yaw, pitch, roll);

#endif
  }
}


void setup()
{
  vars.initializeOffsets();
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  vars.devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(159);
  mpu.setYGyroOffset(14);
  mpu.setZGyroOffset(36);
  mpu.setZAccelOffset(1353); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (vars.devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    vars.mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    vars.dmpReady = true;

    // get expected DMP packet size for later comparison
    vars.packetSize = mpu.dmpGetFIFOPacketSize();
  } //there was else here
  for (size_t i = 0; i < 300; i++)
  {
    readAngle();
  }
  vars.setAngleOffsets();
  //SERVO setup
  y_servo.attach(Y_SERVO_PIN);
  x_servo.attach(X_SERVO_PIN);
  y_servo.write(vars.servoBase);
  x_servo.write(vars.servoBase);

  yawPID.SetMode(AUTOMATIC);
  ptr = ms_init(SGA);
  if (ptr == NULL)
    Serial.println("No memory");
}

void printYPR()
{
  String str = "Yaw: " + (String)yaw + "\t" + "Pitch: " + (String)pitch + "\t" + "Roll: " + (String)roll;
  String str2 = (String)yaw + "," + (String)pitch + "," + (String)roll;
  Serial.println(str2);
}
void loop()
{

  readAngle();

  if(yawPID.Compute()){
    //moveServo(x_servo, vars.ypr[0]);
    //moveServo(y_servo, vars.ypr[1]);
    
  }
}

#include "Gyroscope.h"
#include "Altimeter.h"
#include "PID_v1.h"
#include "Actuator.h"
#include "DataLogger.h"

#define YAW_SERVO_PIN 9   // Y axis
#define PITCH_SERVO_PIN 8 // X axis
#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards
#define MIN_SERVO 1200    // 60°
#define MAX_SERVO 1800    //120°

Actuator yaw_servo;
Actuator pitch_servo;
Gyroscope gyro;
Altimeter altimeter;
DataLogger dataLogger;
double yaw_servoPID_out, pitch_servoPID_out;
double yawSetpoint = 0.0, pitchSetpoint = 0.0; //values to read to correct rocket orientation
double kp = 50, kd = 0.25, ki = 1;
double yaw_pid = 0, pitch_pid = 0;
PID yawPID(&yaw_pid, &yaw_servoPID_out, &yawSetpoint, kp, ki, kd, DIRECT);
PID pitchPID(&pitch_pid, &pitch_servoPID_out, &pitchSetpoint, kp, ki, kd, DIRECT);

void setup()
{
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(9600); //38400 looks fine, 115200 resets the board

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  gyro.setup(INTERRUPT_PIN);
  altimeter.setup();
  dataLogger.setup();
  //SERVO setup
  pitch_servo.setupServo(PITCH_SERVO_PIN);
  yaw_servo.setupServo(YAW_SERVO_PIN);
  //PID
  yawPID.SetOutputLimits(MIN_SERVO, MAX_SERVO);
  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(MIN_SERVO, MAX_SERVO);
  pitchPID.SetMode(AUTOMATIC);
}

void loop()
{

  gyro.readAngle();
  altimeter.getRocketAltitude();
  //update PID input
  VectorFloat eulers = gyro.getEuler();
  pitch_pid = eulers.x * 55;
  yaw_pid = eulers.y * 55;
  if (yawPID.Compute())
  {
    yaw_servo.moveServo(yaw_servoPID_out);
  }
  if (pitchPID.Compute())
  {
    pitch_servo.moveServo(pitch_servoPID_out);
  }

  dataLogger.storeData(gyro, altimeter, yaw_servoPID_out, pitch_servoPID_out, 100L); //will work on error handling later
}

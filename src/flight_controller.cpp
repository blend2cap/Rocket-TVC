#include "Gyroscope.h"
#include "Altimeter.h"
#include "PID_v1.h"
#include "Actuator.h"
#include "DataLogger.h"
#include "LogTimer.hpp"

#define YAW_SERVO_PIN 9   // Y axis
#define PITCH_SERVO_PIN 8 // X axis
#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards
#define MIN_SERVO 1200    // 60°
#define MAX_SERVO 1800    //120°

Actuator yaw_servo;
Actuator pitch_servo;
Gyroscope gyro;
Altimeter altimeter;
DataLogger dataLogger(&gyro, &altimeter);
double yaw_servoPID_out, pitch_servoPID_out;
double yawSetpoint = 0.0, pitchSetpoint = 0.0; //values to read to correct rocket orientation
const double kp = 50, kd = 0.25, ki = 1;
double yaw_pid = 0, pitch_pid = 0;
PID yawPID(&yaw_pid, &yaw_servoPID_out, &yawSetpoint, kp, ki, kd, DIRECT);
PID pitchPID(&pitch_pid, &pitch_servoPID_out, &pitchSetpoint, kp, ki, kd, DIRECT);
//LogTimer *logTimer;

void Test_Strumentation(Gyroscope &gyro, Altimeter &altimeter, Actuator &pitch, Actuator &yaw, DataLogger &datalog);

void setup()
{
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(9600); //38400 looks fine, 115200 resets the board
  //Timer::initCountDown(60); //init T- 60 seconds

  // logTimer->initLogTimer();
  LogTimer::initLogT();
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
  //end setup
  //Start testing
  Test_Strumentation(gyro, altimeter, pitch_servo, yaw_servo, dataLogger);
}

void loop()
{

  gyro.readAngle();
  altimeter.getRocketAltitude();
  //update PID input
  VectorFloat eulers = gyro.getEuler();
  pitch_pid = eulers.x * 55; //*55 for debug
  yaw_pid = eulers.y * 55;
  if (yawPID.Compute())
  {
    yaw_servo.moveServo(yaw_servoPID_out);
  }
  if (pitchPID.Compute())
  {
    pitch_servo.moveServo(pitch_servoPID_out);
  }

  //dataLogger.storeData(yaw_servoPID_out, pitch_servoPID_out, logTimer->getLogTimer()); //will work on error handling later
  dataLogger.storeData(yaw_servoPID_out, pitch_servoPID_out, LogTimer::getT()); //will work on error handling later
}

void Test_Strumentation(Gyroscope &gyro, Altimeter &altimeter, Actuator &pitch, Actuator &yaw, DataLogger &datalog)
{
  Timer tim = Timer(5000, 100);
  uint16_t state = 0;
  uint8_t cont = 0;
  float util;

  for (uint8_t iter = 0; iter < 4; iter++)
  {
    tim.execute_for([&] { state += gyro.readAngle(); cont++; });
    //if more than 25% of reads fails
    util = float(state) / float(cont);
    if (util > 0.25f)
    {
      // Serial.print(F("Gyroscope error: "));
      // Serial.println(util);
      dataLogger.logError("Gyroscope error, failed " + util, LogTimer::getT());
      continue;
    }

    tim.execute_for([&] { util = altimeter.getRocketAltitude(); });
    if (util < 0.f)
    {
      // Serial.println(F("Looks like we're diving!"));
      dataLogger.logError("Altimeter error, altitude recorded: " + util, LogTimer::getT());
      continue;
    }
    if (pitch.checkServo() != 0)
    {
      // Serial.println(F("Pitch servo failed"));
      dataLogger.logError("Pitch servo didn't respond", LogTimer::getT());
      continue;
    }
    if (yaw.checkServo() != 0)
    {
      // Serial.println(F("Yaw servo failed"));
      dataLogger.logError("Yaw servo didn't respond", LogTimer::getT());
      continue;
    }
    if (datalog.check() != 0)
    {
      // Serial.println(F("Cannot save flight data"));
      dataLogger.logError("Cannot save flight data", LogTimer::getT());
      continue;
    }
  }
}
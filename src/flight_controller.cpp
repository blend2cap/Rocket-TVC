#include <Sensor.h>
#include "Servo.h"
#include "PID_v1.h"
#include "Actuator.h"

#define YAW_SERVO_PIN 9   // Y axis
#define PITCH_SERVO_PIN 8 // X axis
#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards

Actuator yaw_servo;
Actuator pitch_servo;
Gyroscope gyro;
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

  //SERVO setup
  yaw_servo.setupServo(YAW_SERVO_PIN);
  //PID
  yawPID.SetOutputLimits(1200, 1800); // 90°+-30°
  yawPID.SetMode(AUTOMATIC);
}

//maps input angle in deg to PWM

void loop()
{

  gyro.readAngle();
  //update PID input
  yaw_pid = gyro.getEuler().y * 55;
  //Serial.println(gyro.log_raw_quaternion());
  //Serial.println(gyro.log_euler());
  //Serial.println(gyro.log_Orientation());
  //Serial.println(gyro.log_euler());
  if (yawPID.Compute())
  {
    //Print current val vs setpoint
    Serial.println((String)gyro.getEuler().y + ",  " + (String)yaw_servoPID_out);
    //Serial.println("Yaw PID: " + (String)yaw_servoPID_out);
    // Serial.println("Euler:  " + (String)gyro.log_euler());
    // int angle = map_angle(yaw_servoPID_out);
    // Serial.println(angle);
    // yaw_servo.moveServo(round(yaw_servoPID_out * 10));
    // Serial.println(yaw_servo.logServoPos());
  }
  // yaw_servo.moveServo(1700);
  // yaw_servo.moveServo(1000);
  // yaw_servo.moveServo(2000);
  yaw_servo.moveServo(yaw_servoPID_out);
  if (pitchPID.Compute())
  {
    //Serial.println("Pitch: " + (String)pitchSetpoint);
  }
}

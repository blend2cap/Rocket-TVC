#include <Sensor.h>
#include "Servo.h"
#include "PID_v1.h"

Servo yaw_servo;
Servo pitch_servo;
Gyroscope gyro;
double yaw_servoPID_val, pitch_servoPID_val;
double yawSetpoint = 0.0, pitchSetpoint = 0.0; //values to read to correct rocket orientation
double kp = 1, kd = 0.25, ki = 0.05;
double yaw_pid, pitch_pid;
uint16_t *yaw_ptr, *pitch_ptr;
PID yawPID(&yaw_pid, &yaw_servoPID_val, &yawSetpoint, kp, ki, kd, DIRECT);
PID pitchPID(&pitch_pid, &pitch_servoPID_val, &pitchSetpoint, kp, ki, kd, DIRECT);

#define YAW_SERVO_PIN 9   // Y axis
#define PITCH_SERVO_PIN 8 // X axis
#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards
#define SERVO_BASE 90

void servoReset(Servo &servo, int pin)
{
  servo.attach(pin);
  servo.write(180);
  servo.write(SERVO_BASE);
}

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
  servoReset(yaw_servo, YAW_SERVO_PIN);
  servoReset(pitch_servo, YAW_SERVO_PIN);
  //PID
  yawPID.SetMode(AUTOMATIC);
}

void loop()
{

  gyro.readAngle();
  //Serial.println(gyro.log_raw_quaternion());
  //Serial.println(gyro.log_euler());
  //Serial.println(gyro.log_Orientation());
  if (yawPID.Compute())
  {
    //Serial.println("Yaw: " + (String)yawSetpoint);
  }
  if (pitchPID.Compute())
  {
    //Serial.println("Pitch: " + (String)pitchSetpoint);
  }
}

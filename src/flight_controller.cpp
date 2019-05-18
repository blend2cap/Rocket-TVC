//#include "I2Cdev.h"
#include "Servo.h"
/*
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
*/
#include "PID_v1.h"
#include <microsmooth.h>
#include <Sensor.h>
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
Servo y_servo;
Servo x_servo;
Gyroscope gyro;
double yaw_servoPIDvalue;
double Setpoint = 0;
double kp = 1, kd = 0.25, ki = 0.05;
double yaw_pid, pitch_pid, roll_pid;
uint16_t *ptr;
PID yawPID(&yaw_pid, &yaw_servoPIDvalue, &Setpoint, kp, ki, kd, DIRECT);

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

void setup()
{
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
  gyro.setup(INTERRUPT_PIN);
  //SERVO setup
  y_servo.attach(Y_SERVO_PIN);
  x_servo.attach(X_SERVO_PIN);
  y_servo.write(90);
  x_servo.write(90);
  //PID
  yawPID.SetMode(AUTOMATIC);
  //filtering
  ptr = ms_init(SGA);
  if (ptr == NULL)
    Serial.println("No memory");
}

void printYPR()
{
  String str = "Yaw: " + (String)yaw_pid + "\t" + "pitch_pid: " + (String)pitch_pid + "\t" + "roll_pid: " + (String)roll_pid;
  String str2 = (String)yaw_pid + "," + (String)pitch_pid + "," + (String)roll_pid;
  Serial.println(str2);
}
void loop()
{

  gyro.readAngle();

  if (yawPID.Compute())
  {
  }
}

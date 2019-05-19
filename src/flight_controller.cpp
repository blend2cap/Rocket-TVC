#include <Sensor.h>
#include "Servo.h"
#include <vector>
#include "PID_v1.h"

Servo yaw_servo;
Servo pitch_servo;
Gyroscope gyro;
double yaw_servoPID_val, pitch_servoPID_val;
double yawSetpoint = 0, pitchSetpoint = 0;
double kp = 1, kd = 0.25, ki = 0.05;
double yaw_pid, pitch_pid;
uint16_t *yaw_ptr, *pitch_ptr;
PID yawPID(&yaw_pid, &yaw_servoPID_val, &yawSetpoint, kp, ki, kd, DIRECT);
PID pitchPID(&pitch_pid, &pitch_servoPID_val, &pitchSetpoint, kp, ki, kd, DIRECT);

#define YAW_SERVO_PIN 9   // Y axis
#define PITCH_SERVO_PIN 8 // X axis
#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards
#define SERVO_BASE 90

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
  yaw_servo.attach(YAW_SERVO_PIN);
  pitch_servo.attach(PITCH_SERVO_PIN);
  yaw_servo.write(SERVO_BASE);
  pitch_servo.write(SERVO_BASE);
  //PID
  yawPID.SetMode(AUTOMATIC);
  /* //filtering
  yaw_ptr = ms_init(SGA);
  pitch_ptr = ms_init(SGA);
  if (yaw_ptr == NULL or pitch_ptr == NULL)
    Serial.println("No memory for filters");
    */
}

void log_csv(std::vector<double> &data)
{
  String l;
  for (double x : data)
  {
    l += (String)x + ",";
  }
  Serial.println(l);
}

void loop()
{

  gyro.readAngle();

  std::vector<double> log_data = {gyro.yaw, gyro.pitch, gyro.roll};

  if (yawPID.Compute())
  {
  }
  if (pitchPID.Compute())
  {
    //move pitch servo
  }
  log_data.push_back(yawSetpoint);
  log_data.push_back(pitchSetpoint);
  log_csv(log_data);
}

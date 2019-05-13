#include<Sensor.h>
class Gyroscope : public Sensor
{

   
protected:
    double yaw;
    double pitch;
    double roll;
    double yaw_offset;
    double pitch_offset;
    double roll_offset;

public:
    Gyroscope();
    ~Gyroscope();

    void setAngleOffsets(double yaw, double pitch, double roll);
};

Gyroscope::Gyroscope()
{
    this->yaw = 0;
    this->pitch = 0;
    this->roll=0;
    this->yaw_offset=0;
    this->pitch_offset=0;
    this->roll_offset=0;
}

Gyroscope::~Gyroscope()
{
}



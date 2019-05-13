#include<Arduino.h>
class Sensor
{
protected:
    String id;
public:
    Sensor(String id="Sensor_001") {}
    ~Sensor() {}

    String getID(){return this->id;}
    inline void setID(String ID){this->id=ID;}

};

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



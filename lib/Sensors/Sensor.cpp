#include<Gyroscope.h>

void Gyroscope::setAngleOffsets(double yaw, double pitch, double roll){
    this->yaw_offset = yaw;
    this->pitch_offset=pitch;
    this->roll_offset=roll;
}

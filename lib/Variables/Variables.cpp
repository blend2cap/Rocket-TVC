#include "Variables.h"

void Variables::setDoubleAngles(double &yaw, double &pitch, double &roll){
  yaw = ypr[0];
  pitch = ypr[1];
  roll = ypr[2];
}

void Variables::initializeOffsets(){
  angleOffsets[0]=0;
  angleOffsets[1]=0;
  angleOffsets[2]=0;
}

void Variables::setAngleOffsets( ){
  angleOffsets[0] = -ypr[0];
  angleOffsets[1] = -ypr[1];
  angleOffsets[2]= -ypr[2];
}

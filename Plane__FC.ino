#include "SBUS_.h"
#include "PID.h"
#include "IMU.h"


void setup(){
  sbus_setup();
  IMUsetup();
  IMUcalibrate(&offset_x, &offset_y, &offset_z);
  servo_setup();
}

void loop(){
  sbus_read();
  IMUdata(&offset_x, &offset_y, &offset_z);
}

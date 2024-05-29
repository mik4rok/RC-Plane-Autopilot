#include <sbus.h>
#include <HardwareSerial.h>

float offset_x = 0.0; //Set Variables for main code 
float offset_y = 0.0;
float offset_z = 0.0;

int Thr = 0;
int Ail = 0;
int Ele = 0;
int Rud = 0;
int ch5 = 0;
int failsafe = 1;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusData data;



void sbus_setup() {
  /* Serial to display data */
  Serial.begin(115200);
  /* Begin the SBUS communication */
  sbus_rx.Begin();
}

void sbus_read() {
  if (sbus_rx.Read()) {
    /* Sbus received data */
    data = sbus_rx.data();
    /* Display the received data */
    // for (int8_t i = 0; i < data.NUM_CH; i++) {
    //   Serial.print(data.ch[i]);
    // Serial.print("\t");
    // }

    int ch1 = data.ch[0];

    ch5 = data.ch[4];
    failsafe = data.failsafe;
    

    // Map values of Ail Ele to values of Angle input
    Thr = map(data.ch[0], 172, 1811, 0, 150); // controls the motor power
    Ail = map(data.ch[1], 172, 1811, -30, 30); // controls the max bank angle of the heli swash
    Ele = map(data.ch[2], 172, 1811, -30, 30); // controls the max pitch anglke of the heli swash 
    Rud = map(data.ch[3], 172, 1811, -50, 50); // usign any arbitrary value for rudder, I dont think this matters



    //Serial.print(Thr);
    //Serial.println(Ele); //prints degrees of input needed
    
    
    // Serial.print(ch1);
    // Serial.print("\t");
     //Serial.println(ch5);
    // Serial.print("\t");
    // Serial.println(data.failsafe);
    // IMUdata();

    /* Set the SBUS TX data to the received data */
  }
}

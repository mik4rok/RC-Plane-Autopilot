#include <math.h>
#include <Servo.h>
Servo AilServo;
Servo EleServo;
Servo YawServo;

// Define PID constants ROLL
#define KRP 3.25
#define KRI 0.0012
#define KRD 0.3

// Define PID constants PITCH
#define KPP 3.25
#define KPI 0.0012
#define KPD 0.3

// Define PID constants YAW
#define KYP 15
#define KYI 0.01
#define KYD 0.15


#define RollRate 1;
#define PitchRate 1;
#define YawRate 0.08;

// Define control limits
#define MAX_PWM 255
#define MIN_PWM 0

float roll_integral = 0;
float prev_roll_error = 0;  // initialize to 0 as integral grows and reduced with time
float pitch_integral = 0;
float prev_pitch_error = 0;
float yaw_integral = 0;
float prev_yaw_error = 0;



void servo_setup() {
  AilServo.attach(8);
  EleServo.attach(9);
  YawServo.attach(10);
}


void sbus_to_pid(float &orientation_z, float &orientation_y, int &Thr, int &Ail, int &Ele, int &Rud, int &ch5, int &failsafe, float &rate_pitch, float &rate_roll, float &rate_yaw) {


  // calculations to output deg/s from radio angle input


  rate_roll = Ail * RollRate;
  rate_pitch = Ele * PitchRate;
  rate_yaw = Rud *YawRate
    rate_yaw = rate_yaw;


  // Serial.print(rate_pitch);
  // Serial.print(" ");
  // Serial.print(rate_yaw);
  // Serial.print(" ");
  // Serial.println(rate_roll);
}


// Update PID controller
void pid_update(float &orientation_z, float &orientation_y, float &gyro_z, float target_roll, float target_pitch, float target_yaw, float &roll_output, float &pitch_output, float &yaw_output, int dt) {

  float roll_error = target_roll - orientation_z;    //might need to be changed based on gyro orientation
  float pitch_error = target_pitch - orientation_y;  // might need to be changed based on gyro orientation
  float yaw_error = target_yaw - gyro_z;             // might need to be changed based on gyro orientation



  roll_output = KRP * roll_error + KRI * roll_integral + KRD * ((roll_error - prev_roll_error) / dt);
  pitch_output = KPP * pitch_error + KPI * pitch_integral + KPD * ((pitch_error - prev_pitch_error) / dt);
  yaw_output = KYP * yaw_error + KYI * yaw_integral + KYD * ((yaw_error - prev_yaw_error) / dt);

  roll_output = constrain(roll_output, -90, 90);
  pitch_output = constrain(pitch_output, -90, 90);
  yaw_output = constrain(yaw_output, -90, 90);


  // Update integral terms
  roll_integral += roll_error * dt;
  pitch_integral += pitch_error * dt;
  yaw_integral += yaw_error * dt;

  roll_integral = constrain(roll_integral, -90, 90);
  pitch_integral = constrain(pitch_integral, -90, 90);
  yaw_integral = constrain(yaw_integral, -90, 90);

  // Update previous error terms
  prev_roll_error = roll_error;
  prev_pitch_error = pitch_error;
  prev_yaw_error = yaw_error;

  //Serial.println("Roll Gyro: " + String(orientation_z));
  //Serial.println("Pitch Gyro: " + String(orientation_y));


  //Serial.println("Yaw: " + String(gyro_z) + " Yaw Error: " + String(yaw_error) + " Yaw Output: " + String(yaw_output));  //debugging pid code
}

// Main function
void PIDmain(float roll_output, float pitch_output, float yaw_output, float Thr) {
  // Calculate PWM values

  int roll_pwm = map(roll_output, -90, 90, 1800, 1200);
  int pitch_pwm = map(pitch_output, -90, 90, 1200, 1800);
  int yaw_pwm = map(yaw_output, -100, 100, 1000, 600);
  int yaw_pwm_mix = yaw_pwm + (Thr * 0.62); //mixing thr 
  yaw_pwm_mix = constrain(yaw_pwm, 600, 1000);



  if (failsafe == 0 && ch5 > 800) {
    if (Thr > 1) {
      analogWrite(0, yaw_pwm);
    } else {
      analogWrite(0, 0);
    }

    analogWrite(1, Thr);
    AilServo.writeMicroseconds(roll_pwm);   // sets the servo position to its minimum
    EleServo.writeMicroseconds(pitch_pwm);  // sets the servo position to its minimum
    YawServo.writeMicroseconds(yaw_pwm);  // sets the servo position to its minimum
  } else {
    analogWrite(0, 0);
    analogWrite(1, 0);
    AilServo.writeMicroseconds(1500);  // sets the servo position to its minimum
    EleServo.writeMicroseconds(1500);  // sets the servo position to its minimum
    YawServo.writeMicroseconds(1500);  // sets the servo position to its minimum
  }



  Serial.println("roll PWM 1000-2000: " + String(roll_pwm) + " Roll PID: " + String(roll_output));
  Serial.println("Pitch PWM 1000-2000: " + String(pitch_pwm) + " Roll PID: " + String(pitch_output));

  Serial.println("Yaw PWM 0-1024: " + String(yaw_pwm) + " Yaw PID: " + String(yaw_output));
}

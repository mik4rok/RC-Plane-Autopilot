// Basic on demo for accelerometer readings from Adafruit MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Set the delay between fresh samples */
uint16_t SAMPLERATE_DELAY_MS = 10;

// Check I2C device address and correct line below (by default address is 0x68)
Adafruit_MPU6050 mpu;

unsigned long lastTime = 0;

void IMUsetup(void) {
  // Try to initialize!
  if (!mpu.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      ;
    }

    lastTime = millis();
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

// Declare a function to return sensor data.
void getSensorData(float &gyro_x, float &gyro_y, float &gyro_z, float &orientation_x, float &orientation_y, float &orientation_z) {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 // Store gyro and orientation data in the function parameters.
  gyro_x = g.gyro.x;
  gyro_y = g.gyro.y;
  gyro_z = g.gyro.z;
  orientation_x = a.acceleration.x;
  orientation_y = a.acceleration.y;
  orientation_z = a.acceleration.z;
}

//Getting offset fo accelerator
void IMUcalibrate(float *offset_x, float *offset_y, float *offset_z) {
  float gyro_x, gyro_y, gyro_z, GyX, GyY, GyZ;

  float gx = 0.0; float gy = 0.0; float gz = 0.0; //gyro offsets sum
  
  for (int i = 0; i < 500; i++) {
    getSensorData(gyro_x, gyro_y, gyro_z, GyX, GyY, GyZ);
    gx += GyX; gy += GyY; gz += GyZ;
  }
  *offset_x = gx / 500.0;
  *offset_y = gy / 500.0;
  *offset_z = gz / 500.0;

  Serial.print("Gyro offsets ");
  Serial.print(*offset_x);
  Serial.print(", ");
  Serial.print(*offset_y);
  Serial.print(", ");
  Serial.println(*offset_z);
  Serial.println("corrected gyro values follow");
}
//

float roll_output = 0, pitch_output = 0, yaw_output = 0;

void IMUdata() {

  if (millis() - lastTime >= SAMPLERATE_DELAY_MS) {
    lastTime = millis();
    float gyro_x, gyro_y, gyro_z, orientation_x, orientation_y, orientation_z;
    float rate_pitch = 0.0, rate_roll = 0.0, rate_yaw = 0.0;  // output from !


    // Call the getSensorData function to get the latest gyro and orientation data.
    getSensorData(gyro_x, gyro_y, gyro_z, orientation_x, orientation_y, orientation_z);

    // take input data and update it
    // 0.0 -> desired roll rate deg/s


    sbus_to_pid(orientation_z, orientation_y, Thr, Ail, Ele, Rud, ch5, failsafe, rate_pitch, rate_roll, rate_yaw);

    pid_update(orientation_z, orientation_y, gyro_z, rate_roll, rate_pitch, rate_yaw, roll_output, pitch_output, yaw_output, SAMPLERATE_DELAY_MS);

    PIDmain(roll_output, pitch_output, yaw_output, Thr);




    // // Print the gyro and orientation data.
    // Serial.print("Gyro: X:");
    // Serial.print(gyro_x);
    // Serial.print(" Y:");
    // Serial.print(gyro_y);
    // Serial.print(" Z:");
    // Serial.print(gyro_z);
    // Serial.print(" | Orientation: X:");
    // Serial.print(orientation_x);
    // Serial.print(" Y:");
    // Serial.print(orientation_y);
    // Serial.print(" Z:");
    // Serial.println(orientation_z);
  }
}


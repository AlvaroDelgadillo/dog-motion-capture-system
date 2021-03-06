//Deploy with Arduino Mega

//include libraries
#include <MPU6050_9Axis_MotionApps41.h>
#include <MPU6050.h>
#include <math.h>
#include "I2Cdev.h"

#include <Wire.h>
#include<Kalman.h>
#define RESTRICT_PITCH

// these provide us with processed angles
Kalman kalmanX1;
Kalman kalmanY1;
Kalman kalmanX2;
Kalman kalmanY2;

double accX1, accY1, accZ1, accX2, accY2, accZ2;
double gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2;
int16_t tempRaw;

double gyroX1angle, gyroY1angle, gyroX2angle, gyroY2angle; // Angle calculate using the gyro only
double compAngleX1, compAngleY1, compAngleX2, compAngleY2; // Calculated angle using a complementary filter
double kalAngleX1, kalAngleY1, kalAngleX2, kalAngleY2; // Calculated angle using a Kalman filter

uint32_t timer;
int16_t ax1, ax2, ay1, ay2, az1, az2, gx1, gx2, gy1, gy2, gz1, gz2;
// make the gyros
MPU6050 gyro1(0x68);    //Port MPU6050-1  Default
MPU6050 gyro2(0x69);    //Port MPU6050-2

void setup() {
  // pinMode(2, OUTPUT); //s0
  // pinMode(3, OUTPUT); //s1
  // pinMode(4, OUTPUT); //s2
  // pinMode(5, OUTPUT); //s3

  Wire.begin();
  Serial.begin(115200);
  gyro1.initialize();
  gyro2.initialize();
  // put your setup code here, to run once:
  delay(100);

  // this grabs the initial raw values from the gyros
  gyro1.getAcceleration(&ax1, &ay1, &az1);
  gyro1.getRotation(&gx1, &gy1, &gz1);
  accX1 = ax1;
  accZ1 = az1;
  accY1 = ay1;
  gyroX1 = gx1;
  gyroY1 = gy1;
  gyroZ1 = gz1;
  gyro2.getAcceleration(&ax2, &ay2, &az2);
  gyro2.getRotation(&gx2, &gy2, &gz2);
  accX2 = (double)ax2;
  accZ2 = (double)az2;
  accY2 = (double)ay2;
  gyroX2 = gx2;
  gyroY2 = gy2;
  gyroZ2 = gz2;

  // this starts the processing of the data
  double roll1 = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double roll2 = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;

  kalmanX1.setAngle(roll1);
  kalmanY1.setAngle(pitch1);
  kalmanX2.setAngle(roll2);
  kalmanY2.setAngle(pitch2);
  gyroX1angle = roll1;
  gyroY1angle = pitch1;
  compAngleX1 = roll1;
  compAngleY1 = pitch1;
  gyroX2angle = roll2;
  gyroY2angle = pitch2;
  compAngleX2 = roll2;
  compAngleY2 = pitch2;
  timer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  // repeat of the set up code
  gyro1.getAcceleration(&ax1, &ay1, &az1);
  gyro1.getRotation(&gx1, &gy1, &gz1);
  accX1 = ax1;
  accZ1 = az1;
  accY1 = ay1;
  gyroX1 = gx1;
  gyroY1 = gy1;
  gyroZ1 = gz1;
  gyro2.getAcceleration(&ax2, &ay2, &az2);
  gyro2.getRotation(&gx2, &gy2, &gz2);
  accX2 = ax2;
  accZ2 = az2;
  accY2 = ay2;
  gyroX2 = gx2;
  gyroY2 = gy2;
  gyroZ2 = gz2;
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  double roll1 = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double roll2 = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double gyroX1rate = gyroX1 / 131.0; // Convert to deg/s
  double gyroY1rate = gyroY1 / 131.0; // Convert to deg/s
  double gyroX2rate = gyroX2 / 131.0; // Convert to deg/s
  double gyroY2rate = gyroY2 / 131.0; // Convert to deg/s

  if ((roll1 < -90 && kalAngleX1 > 90) || (roll1 > 90 && kalAngleX1 < -90)) {
    kalmanX1.setAngle(roll1);
    compAngleX1 = roll1;
    kalAngleX1 = roll1;
    gyroX1angle = roll1;
  } else {
    kalAngleX1 = kalmanX1.getAngle(roll1, gyroX1rate, dt);
  }
  if (abs(kalAngleX1) > 90)
    gyroY1rate = -gyroY1rate;
  kalAngleY1 = kalmanY1.getAngle(pitch1, gyroY1rate, dt);
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroX2angle = roll2;
  } else {
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroX2rate, dt);
  }
  if (abs(kalAngleX2) > 90)
    gyroY2rate = -gyroY2rate;
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroY2rate, dt);
  gyroX1angle += gyroX1rate * dt; // Calculate gyro angle without any filter
  gyroY1angle += gyroY1rate * dt;
  gyroX2angle += gyroX2rate * dt; // Calculate gyro angle without any filter
  gyroY2angle += gyroY2rate * dt;
  compAngleX1 = 0.93 * (compAngleX1 + gyroX1rate * dt) + 0.07 * roll1; // Calculate the angle using a Complimentary filter
  compAngleY1 = 0.93 * (compAngleY1 + gyroY1rate * dt) + 0.07 * pitch1;

  compAngleX2 = 0.93 * (compAngleX2 + gyroX2rate * dt) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroY2rate * dt) + 0.07 * pitch2;
  if (gyroX1angle < -180 || gyroX1angle > 180)
    gyroX1angle = kalAngleX1;
  if (gyroY1angle < -180 || gyroY1angle > 180)
    gyroY1angle = kalAngleY1;
  if (gyroX2angle < -180 || gyroX2angle > 180)
    gyroX2angle = kalAngleX2;
  if (gyroY2angle < -180 || gyroY2angle > 180)
    gyroY2angle = kalAngleY2;

  // Send the all of the gyro data and sensor data to the serial port
  //first set print
  Serial.print(roll1); Serial.print("\t");
  Serial.print(gyroX1angle); Serial.print("\t");
  Serial.print(compAngleX1); Serial.print("\t");
  Serial.print(kalAngleX1); Serial.print("\t");
  Serial.print("\t");
  Serial.print(pitch1); Serial.print("\t");
  Serial.print(gyroY1angle); Serial.print("\t");
  Serial.print(compAngleY1); Serial.print("\t");
  Serial.print(kalAngleY1); Serial.print("\t");
  Serial.print("\t");
  // second set print
  Serial.print(roll2); Serial.print("\t");
  Serial.print(gyroX2angle); Serial.print("\t");
  Serial.print(compAngleX2); Serial.print("\t");
  Serial.print(kalAngleX2); Serial.print("\t");
  Serial.print("\t");
  Serial.print(pitch2); Serial.print("\t");
  Serial.print(gyroY2angle); Serial.print("\t");
  Serial.print(compAngleY2); Serial.print("\t");
  Serial.print(kalAngleY2); Serial.print("\t");
  Serial.print("\t");

  delay(2);
}

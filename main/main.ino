#include <Wire.h>
#include <Kalman.h>

#define CHA_L 2
#define CHB_L 4
#define CHA_R 7
#define CHB_R 8
#define wheelRadius 0.0335
#define CPR 853
#define PI_val 3.14159 

volatile int count_L = 0;
volatile double wheelDistance_L;
volatile int count_R = 0;
volatile double wheelDistance_R;
volatile double distance;

const int MPU_addr=0x68;  // I2C address of the MPU-6050

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
volatile int16_t accX, accY, accZ;
volatile int16_t gyroX, gyroY, gyroZ;
volatile int16_t tempRaw;

volatile double roll, pitch;

volatile double gyroXrate, gyroYrate;

volatile double gyroXangle, gyroYangle; // Angle calculate using the gyro only
volatile double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

volatile uint32_t timer;
volatile double dt;

void setup(){
  // Encoder setup.
  pinMode(CHA_L, INPUT);
  pinMode(CHB_L, INPUT);
  pinMode(CHA_R, INPUT);
  pinMode(CHB_R, INPUT);

  attachInterrupt(0, interruptionFunction_L, RISING);
  attachInterrupt(0, interruptionFunction_R, RISING);

  // MPU6050 setup.
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  delay(100);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers

  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)  

  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();

}

void loop(){
  wheelDistance_L = count_L * 2 * PI_val * wheelRadius / CPR;
  wheelDistance_R = count_R * 2 * PI_val * wheelRadius / CPR;
  distance = (wheelDistance_L + wheelDistance_R)/2;
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers

  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  
  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter


  //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;
  
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  Serial.print("Kalman pitch: ");
  Serial.print(kalAngleY);
  Serial.print("\t");
  Serial.print("Kalman roll: ");
  Serial.print(kalAngleX);
  Serial.print("\t");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("\t");
  Serial.print("Sample time: ");
  Serial.print(dt);
  Serial.println(" s");

}

void interruptionFunction_L() {
  if (digitalRead(CHA_L) && !digitalRead(CHB_L)) {
    count_L++ ;
  }
  if (digitalRead(CHA_L) && digitalRead(CHB_L)) {
    count_L-- ;
  } 
}

void interruptionFunction_R() {
  if (digitalRead(CHA_R) && !digitalRead(CHB_R)) {
    count_R++ ;
  }
  if (digitalRead(CHA_R) && digitalRead(CHB_R)) {
    count_R-- ;
  } 
}

#include <FastIMU.h>
#include <Wire.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define MPU_ADDRESS 0x68

MPU6500 IMU;
calData calib = {0};
AccelData accelData;
GyroData gyroData;

void setup(){
  Serial.begin(115200);
  Serial.println("Serial began");
  while (!Serial){
    delay(10);
    Serial.println("Serial error");
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(40000);

  int err = IMU.init(calib, MPU_ADDRESS);
  if (err != 0){
    Serial.println(err);
    while(1){
      delay(10);
      Serial.println("IMU init error");
    }
  }

  IMU.setAccelRange(4);
  IMU.setGyroRange(500);
}

void loop(){
  IMU.update();

  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  Serial.println("update succesful");

  Serial.print("Accel X: "); Serial.print(accelData.accelX);
  Serial.print("\tY: "); Serial.print(accelData.accelY);
  Serial.print("\tZ: "); Serial.print(accelData.accelZ);
  Serial.println(" g");

  Serial.print("Gyro X: ");  Serial.print(gyroData.gyroX);
  Serial.print("\tY: "); Serial.print(gyroData.gyroY);
  Serial.print("\tZ: "); Serial.print(gyroData.gyroZ);
  Serial.println(" dps");

  Serial.println("--------------------------------------------------");
  delay(200);
}
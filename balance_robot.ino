#include <FastIMU.h>
#include <Wire.h>

//Ku = ultimate gain — the critical proportional gain value that produces sustained, constant-amplitude oscillation
//Pu = ultimate period — the time (in seconds) for one full oscillation cycle at that critical point
class PID {
  public:
    PID(float setpoint, float Ku, float Pu, float output_min, float output_max) : setpoint(setpoint), output_min(output_min), output_max(output_max), sum_integral(0), prev_error(0), prev_reading(0), initialized(false){
      //sum of error for integral gain calculation
      //previous error reading for derivative calculation
      //previous time reading for _ calculation
      ZN(Ku, Pu);
    }

    //ziegler nichols
    void ZN(float Ku, float Pu){
        //proportional gain
        Kp = Ku * 0.6;
        //integral gain
        Ki = Kp * 2.0 / Pu;
        //derivative gain
        Kd = Kp * Pu / 8.0;
    }

    //returns output based on value of reading and change in time
    float update(float reading, float dt){
      if (dt <= 0){
        return 0;
      }

      //calculate error and derivative, update integral sum
      float error = setpoint - reading;
      float derivative = initialized ? (prev_reading - reading) / dt : 0;
      float temp_integral = sum_integral + error * dt;

      //calculate output (clamped)
      float unclamped_output = Kp*error + Ki*temp_integral + Kd*derivative;
      float output = constrain(unclamped_output, output_min, output_max);

      //alter integral sum if needed
      if (unclamped_output >= output_min && unclamped_output <= output_max){
        sum_integral = temp_integral;
      } else if (unclamped_output > output_max && error <= 0){
        sum_integral = temp_integral;
      } else if (unclamped_output < output_min && error >= 0){
        sum_integral = temp_integral;
      }

      //update previous error, reading, and initialized condition
      prev_error = error;
      prev_reading = reading;
      initialized = true;

      return output;
    }

    void setSetpoint(float sp) {setpoint = sp;}
    float getSetpoint() {return setpoint;}

  private:
    float setpoint, Kp, Ki, Kd, output_min, output_max, sum_integral, prev_error, prev_reading;
    bool initialized;
};

#define I2C_SDA 8
#define I2C_SCL 9
#define MPU_ADDRESS 0x68

MPU6500 IMU;
calData calib = {0};
AccelData accelData;
GyroData gyroData;

float gyroBiasX = 0;
float filteredAngle = 0;
unsigned long lastMicros;
float dt;

//adjust Ku (2) and Pu (3) experimentally
PID balance(0.0, 20.0, 0.5, -255, 255);
TickType_t last_wake_time;

const int AIN1 = 16;
const int AIN2 = 17;
const int PWMA = 18;

const int BIN1 = 5;
const int BIN2 = 6;
const int PWMB = 7;

const int STBY = 4;

const int pwmFreq = 10000;
const int pwmResolution = 8;

void setup() {
  last_wake_time = xTaskGetTickCount();

  Serial.begin(115200);
  Serial.println("Serial began");
  while(!Serial){
    delay(10);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  int err = IMU.init(calib, MPU_ADDRESS);
  if (err != 0){
    Serial.println(err);
    while(1){
      delay(10);
    }
  }

  IMU.setAccelRange(4);
  IMU.setGyroRange(500);

  //gyro bias calib (keep robot still!!!)
  const int calibSamples = 500;
  for (int i = 0; i < calibSamples; i++){
    IMU.update();
    IMU.getGyro(&gyroData);
    gyroBiasX += gyroData.gyroX;
    delay(2);
  }
  gyroBiasX /= calibSamples;

  lastMicros = micros();

  float setpointSum = 0;
  const int setpointSamples = 200;
  for (int i = 0; i < setpointSamples; i++){
    filteredAngle = readIMU();
    setpointSum += filteredAngle;
    delay(5);
  }
  float restAngle = setpointSum / setpointSamples;
  balance.setSetpoint(restAngle);

  //config motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  //setup pwm channels
  ledcAttach(PWMA, pwmFreq, pwmResolution);
  ledcAttach(PWMB, pwmFreq, pwmResolution);

  //standby high, not low power mode
  digitalWrite(STBY, HIGH);
}

//micros() instead of hardcoded 0.008
void loop() {
  float angle = readIMU();
  
  //motors stop if robot tips below set angle
  if (abs(angle) > 45){
    stopMotors();
    return;
  }

  float output = balance.update(angle, dt);
  bool forward = true;
  if (output < 0){
    forward = false;
  }
  moveMotor(AIN1, AIN2, PWMA, abs(output), forward);
  moveMotor(BIN1, BIN2, PWMB, abs(output), forward);
  //Serial.println(angle);
  vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(8));
}

float readIMU(){
  unsigned long now = micros();
  dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  float accelAngle = atan2(accelData.accelX, accelData.accelZ) * 180.0/PI;
  float gyroRate = gyroData.gyroX - gyroBiasX;

  float alpha = 0.98;
  filteredAngle = alpha * (filteredAngle + gyroRate * dt) + (1-alpha) * accelAngle;

  return filteredAngle;
}

void moveMotor(int motor1, int motor2, int pwmPin, float speed, boolean forward){
  if (forward){
    digitalWrite(motor1, HIGH);
    digitalWrite(motor2, LOW);
  } else {
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, HIGH);
  }
  ledcWrite(pwmPin, (int)speed);
}

void stopMotors(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}
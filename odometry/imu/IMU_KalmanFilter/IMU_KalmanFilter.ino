#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
uint32_t LoopTimer;

float KalmanAngleRoll = 0, KalmanUncertainityAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertainityAnglePitch = 2*2;
float KalmanAngleYaw = 0, KalmanUncertainityAngleYaw = 2*2;

float Kalman1DOutput[] = {0, 0};

void kalman_1d (float KalmanState, float KalmanUncertainity, float KalmanInput, float KalmanMeasurement)
{
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertainity = KalmanUncertainity + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainity * 1/(1*KalmanUncertainity+3*3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainity = (1 - KalmanGain)*KalmanUncertainity;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainity;
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  AccX = (float)AccXLSB/4096;
  AccY = (float)AccYLSB/4096;
  AccZ = (float)AccZLSB/4096;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
  AngleYaw = atan2(AccX, AccZ) * (180 / 3.142);
}

void setup(){

  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}

void loop() {

  gyro_signals();
  
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertainityAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertainityAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertainityAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertainityAnglePitch = Kalman1DOutput[1];
  
  kalman_1d(KalmanAngleYaw, KalmanUncertainityAngleYaw, RateYaw, AngleYaw);
  KalmanAngleYaw = Kalman1DOutput[0];
  KalmanUncertainityAngleYaw = Kalman1DOutput[1];
  
  Serial.print("Roll =");
  Serial.print(AngleRoll);
  Serial.print(',');
  Serial.print("Pitch =");
  Serial.print(AnglePitch);
  Serial.print(',');
  Serial.print("Yaw =");
  Serial.print(AnglePitch);
  Serial.print(',');
  Serial.println();
  delay(100);
}

/*
===========================================================
ESP32 QUADCOPTER FLIGHT CONTROLLER
===========================================================

FEATURES:
- MPU6050 IMU
- Complementary Filter
- Dual PID Loop
- Receiver PWM Input
- ESC PWM Output
- Arming / Disarming
- Failsafe
- Fixed Loop Timing

BOARD:
ESP32

IMU:
MPU6050

===========================================================
*/

#include <Wire.h>
#include <ESP32Servo.h>

// =========================================================
// LOOP TIME
// =========================================================

const float LOOP_TIME = 0.004f; // 4ms = 250Hz

uint32_t LoopTimer;

// =========================================================
// MPU6050 VARIABLES
// =========================================================

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

float AngleRoll, AnglePitch;

float complementaryAngleRoll = 0;
float complementaryAnglePitch = 0;

// =========================================================
// CALIBRATION VALUES
// =========================================================

float RateCalibrationRoll = 0.27;
float RateCalibrationPitch = -0.85;
float RateCalibrationYaw = -2.09;

float AccXCalibration = 0.03;
float AccYCalibration = 0.01;
float AccZCalibration = -0.07;

// =========================================================
// RECEIVER VARIABLES
// =========================================================

volatile int ReceiverValue[6] = {1500,1500,1000,1500,1500,1500};

volatile uint32_t timer1, timer2, timer3, timer4;

volatile uint32_t lastChannel1 = 0;
volatile uint32_t lastChannel2 = 0;
volatile uint32_t lastChannel3 = 0;
volatile uint32_t lastChannel4 = 0;

// =========================================================
// RECEIVER PINS
// =========================================================

const int channel_1_pin = 34; // Roll
const int channel_2_pin = 35; // Pitch
const int channel_3_pin = 32; // Throttle
const int channel_4_pin = 33; // Yaw

// =========================================================
// ESC + MOTOR SETTINGS
// =========================================================

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

const int ESCfreq = 50;

int ThrottleIdle = 1150;
int ThrottleCutOff = 1000;

// =========================================================
// ARMING
// =========================================================

bool armed = false;

// =========================================================
// PID VALUES
// =========================================================

// ANGLE PID

float PAngleRoll = 2.0;
float IAngleRoll = 0.5;
float DAngleRoll = 0.007;

float PAnglePitch = 2.0;
float IAnglePitch = 0.5;
float DAnglePitch = 0.007;

// RATE PID

float PRateRoll = 0.5;
float IRateRoll = 0.8;
float DRateRoll = 0.003;

float PRatePitch = 0.5;
float IRatePitch = 0.8;
float DRatePitch = 0.003;

float PRateYaw = 2.0;
float IRateYaw = 0.5;
float DRateYaw = 0.0;

// =========================================================
// PID VARIABLES
// =========================================================

float DesiredAngleRoll;
float DesiredAnglePitch;

float DesiredRateRoll;
float DesiredRatePitch;
float DesiredRateYaw;

float InputRoll;
float InputPitch;
float InputYaw;

float InputThrottle;

// ANGLE PID MEMORY

float PrevErrorAngleRoll = 0;
float PrevErrorAnglePitch = 0;

float PrevItermAngleRoll = 0;
float PrevItermAnglePitch = 0;

// RATE PID MEMORY

float PrevErrorRateRoll = 0;
float PrevErrorRatePitch = 0;
float PrevErrorRateYaw = 0;

float PrevItermRateRoll = 0;
float PrevItermRatePitch = 0;
float PrevItermRateYaw = 0;

// =========================================================
// MOTOR OUTPUTS
// =========================================================

float MotorInput1;
float MotorInput2;
float MotorInput3;
float MotorInput4;

// =========================================================
// INTERRUPTS
// =========================================================

void IRAM_ATTR channel1Interrupt()
{
  if (digitalRead(channel_1_pin))
  {
    timer1 = micros();
  }
  else
  {
    ReceiverValue[0] = micros() - timer1;
  }
}

void IRAM_ATTR channel2Interrupt()
{
  if (digitalRead(channel_2_pin))
  {
    timer2 = micros();
  }
  else
  {
    ReceiverValue[1] = micros() - timer2;
  }
}

void IRAM_ATTR channel3Interrupt()
{
  if (digitalRead(channel_3_pin))
  {
    timer3 = micros();
  }
  else
  {
    ReceiverValue[2] = micros() - timer3;
  }
}

void IRAM_ATTR channel4Interrupt()
{
  if (digitalRead(channel_4_pin))
  {
    timer4 = micros();
  }
  else
  {
    ReceiverValue[3] = micros() - timer4;
  }
}

// =========================================================
// MPU6050 READ
// =========================================================

void readMPU()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 14);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096.0;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0;

  // Calibration

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;
}

// =========================================================
// CALCULATE ANGLES
// =========================================================

void calculateAngles()
{
  AngleRoll =
      atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958;

  AnglePitch =
      -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.2958;

  // Complementary Filter

  complementaryAngleRoll =
      0.98 * (complementaryAngleRoll + RateRoll * LOOP_TIME) +
      0.02 * AngleRoll;

  complementaryAnglePitch =
      0.98 * (complementaryAnglePitch + RatePitch * LOOP_TIME) +
      0.02 * AnglePitch;
}

// =========================================================
// PID FUNCTION
// =========================================================

float calculatePID(
    float error,
    float P,
    float I,
    float D,
    float &prevError,
    float &prevIterm)
{
  float Pterm = P * error;

  float Iterm =
      prevIterm +
      I * (error + prevError) * (LOOP_TIME / 2);

  Iterm = constrain(Iterm, -400, 400);

  float Dterm =
      D * ((error - prevError) / LOOP_TIME);

  float output = Pterm + Iterm + Dterm;

  output = constrain(output, -400, 400);

  prevError = error;
  prevIterm = Iterm;

  return output;
}

// =========================================================
// SETUP
// =========================================================

void setup()
{
  Serial.begin(115200);

  // =======================================================
  // RECEIVER INPUTS
  // =======================================================

  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(channel_1_pin),
      channel1Interrupt,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(channel_2_pin),
      channel2Interrupt,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(channel_3_pin),
      channel3Interrupt,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(channel_4_pin),
      channel4Interrupt,
      CHANGE);

  // =======================================================
  // I2C + MPU6050
  // =======================================================

  Wire.begin();
  Wire.setClock(400000);

  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Low Pass Filter

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Accelerometer ±8g

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Gyro ±500 deg/s

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // =======================================================
  // ESC SETUP
  // =======================================================

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  mot1.setPeriodHertz(ESCfreq);
  mot2.setPeriodHertz(ESCfreq);
  mot3.setPeriodHertz(ESCfreq);
  mot4.setPeriodHertz(ESCfreq);

  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);

  // ESC INIT

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);

  delay(3000);

  LoopTimer = micros();
}

// =========================================================
// MAIN LOOP
// =========================================================

void loop()
{
  // =======================================================
  // READ IMU
  // =======================================================

  readMPU();

  calculateAngles();

  // =======================================================
  // RECEIVER INPUTS
  // =======================================================

  DesiredAngleRoll =
      0.1 * (ReceiverValue[0] - 1500);

  DesiredAnglePitch =
      0.1 * (ReceiverValue[1] - 1500);

  InputThrottle = ReceiverValue[2];

  DesiredRateYaw =
      0.15 * (ReceiverValue[3] - 1500);

  // =======================================================
  // ARMING LOGIC
  // =======================================================

  if (InputThrottle < 1050 &&
      ReceiverValue[3] < 1050)
  {
    armed = true;
  }

  if (InputThrottle < 1050 &&
      ReceiverValue[3] > 1900)
  {
    armed = false;
  }

  // =======================================================
  // ANGLE PID
  // =======================================================

  float ErrorAngleRoll =
      DesiredAngleRoll - complementaryAngleRoll;

  float ErrorAnglePitch =
      DesiredAnglePitch - complementaryAnglePitch;

  DesiredRateRoll = calculatePID(
      ErrorAngleRoll,
      PAngleRoll,
      IAngleRoll,
      DAngleRoll,
      PrevErrorAngleRoll,
      PrevItermAngleRoll);

  DesiredRatePitch = calculatePID(
      ErrorAnglePitch,
      PAnglePitch,
      IAnglePitch,
      DAnglePitch,
      PrevErrorAnglePitch,
      PrevItermAnglePitch);

  // =======================================================
  // RATE PID
  // =======================================================

  float ErrorRateRoll =
      DesiredRateRoll - RateRoll;

  float ErrorRatePitch =
      DesiredRatePitch - RatePitch;

  float ErrorRateYaw =
      DesiredRateYaw - RateYaw;

  InputRoll = calculatePID(
      ErrorRateRoll,
      PRateRoll,
      IRateRoll,
      DRateRoll,
      PrevErrorRateRoll,
      PrevItermRateRoll);

  InputPitch = calculatePID(
      ErrorRatePitch,
      PRatePitch,
      IRatePitch,
      DRatePitch,
      PrevErrorRatePitch,
      PrevItermRatePitch);

  InputYaw = calculatePID(
      ErrorRateYaw,
      PRateYaw,
      IRateYaw,
      DRateYaw,
      PrevErrorRateYaw,
      PrevItermRateYaw);

  // =======================================================
  // THROTTLE LIMIT
  // =======================================================

  InputThrottle = constrain(InputThrottle, 1000, 1800);

  // =======================================================
  // MOTOR MIXING (X CONFIGURATION)
  // =======================================================

  // FRONT RIGHT - CCW
  MotorInput1 =
      InputThrottle -
      InputRoll -
      InputPitch -
      InputYaw;

  // REAR RIGHT - CW
  MotorInput2 =
      InputThrottle -
      InputRoll +
      InputPitch +
      InputYaw;

  // REAR LEFT - CCW
  MotorInput3 =
      InputThrottle +
      InputRoll +
      InputPitch -
      InputYaw;

  // FRONT LEFT - CW
  MotorInput4 =
      InputThrottle +
      InputRoll -
      InputPitch +
      InputYaw;

  // =======================================================
  // MOTOR LIMITS
  // =======================================================

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  // =======================================================
  // FAILSAFE
  // =======================================================

  if (ReceiverValue[2] < 900 ||
      ReceiverValue[2] > 2100)
  {
    armed = false;
  }

  // =======================================================
  // DISARM
  // =======================================================

  if (!armed)
  {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;

    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;

    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;

    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;

    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;
  }

  // =======================================================
  // WRITE ESC OUTPUTS
  // =======================================================

  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  // =======================================================
  // DEBUG
  // =======================================================

  /*
  Serial.print(complementaryAngleRoll);
  Serial.print(" ");
  Serial.print(complementaryAnglePitch);
  Serial.print(" ");
  Serial.println(InputThrottle);
  */

  // =======================================================
  // LOOP TIMER
  // =======================================================

  while (micros() - LoopTimer < 4000)
  {
  }

  LoopTimer = micros();
}
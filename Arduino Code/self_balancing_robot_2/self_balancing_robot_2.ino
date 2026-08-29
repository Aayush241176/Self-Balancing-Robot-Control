// Self Balancing Robot - Latching Safety Cutoff + PID Reset
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 24

// ---------- Safety tuning ----------
const double BALANCE_SETPOINT   = 172.8;   // your calibrated upright angle
const double SAFETY_TILT_DEG    = 20.0;    // fault if robot goes beyond this from upright
const double REARM_TILT_DEG     = 4.0;     // must come back within this range to re-arm
const unsigned long REARM_STABLE_MS = 800;  // must stay upright this long before balancing again

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = BALANCE_SETPOINT;
double setpoint = originalSetpoint;
double input, output;

// Adjust these values during tuning
double Kp = 22.0;       // PRESERVED: Your adjusted Kp
double Kd = 1.8;      // PRESERVED: Your adjusted Kd
double Ki = 125.0;    // PRESERVED: Your adjusted Ki

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;

// MOTOR CONTROLLER
int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENB = 10;

LMotorController motorController(
  ENA, IN1, IN2, ENB, IN3, IN4,
  motorSpeedFactorLeft, motorSpeedFactorRight
);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

bool safetyFault = true;              // start disarmed until upright and stable
unsigned long uprightStartMs = 0;

void hardStop()
{
  output = 0;
  motorController.move(0, 0);

  // Hard stop on bridge pins too
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setup()
{
  Serial.begin(115200);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  hardStop();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(MANUAL);          // stay disabled until robot is upright
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  if (!dmpReady) return;

  if (mpuInterrupt || fifoCount >= packetSize)
  {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
      hardStop();
      return;
    }
    else if (mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Current angle
      input = ypr[1] * 180 / M_PI + 180;

      // Angle error from upright
      double angleError = fabs(input - originalSetpoint);

      // 1) Immediate fault if robot is picked up / tilted too far
      if (angleError > SAFETY_TILT_DEG || isnan(input))
      {
        safetyFault = true;
        uprightStartMs = 0;
        pid.SetMode(MANUAL);   // clears PID output history / integral action
        hardStop();
        return;
      }

      // 2) If in fault state, only re-arm when upright and stable
      if (safetyFault)
      {
        if (angleError <= REARM_TILT_DEG)
        {
          if (uprightStartMs == 0)
            uprightStartMs = millis();

          if (millis() - uprightStartMs >= REARM_STABLE_MS)
          {
            safetyFault = false;
            pid.SetMode(AUTOMATIC);   // reinitializes PID
            output = 0;
          }
        }
        else
        {
          uprightStartMs = 0;
        }

        hardStop();
        return;
      }

      // 3) Normal balancing only when armed
      setpoint = originalSetpoint;
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);
    }
  }
}
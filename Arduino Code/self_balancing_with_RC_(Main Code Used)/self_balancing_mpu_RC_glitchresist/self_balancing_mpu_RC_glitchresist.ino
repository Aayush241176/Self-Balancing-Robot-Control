// Self Balancing Robot - Watchdog Protected + Noise Isolated Edition
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <avr/wdt.h> // <--- HARDWARE WATCHDOG LIFELINE

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 18

// ---------- Safety Tuning ----------
const double BALANCE_SETPOINT   = 172.8;   
const double SAFETY_TILT_DEG    = 20.0;    
const double REARM_TILT_DEG     = 4.0;     
const unsigned long REARM_STABLE_MS = 800;  

// ---------- RC Pin Configurations ----------
#define RC_FORWARD_PIN   8
#define RC_BACK_PIN      3
#define RC_LEFT_PIN      12
#define RC_RIGHT_PIN     9

// ---------- Decoupled Directional Tuning ----------
double MAX_FORWARD_ANGLE = 3.8;   
double MAX_BACK_ANGLE    = 4.0;   
double MAX_TURN_SPEED    = 24.0;  

// SMOOTHED: Lowered slightly to eliminate aggressive current spikes during torque changes
double TILT_RAMP_FACTOR = 0.07;   // Anti-spike soft ramp
double TURN_RAMP_FACTOR = 0.15;   

// Compensation for remote control physical stick drops
const unsigned long RC_FWD_HOLD_MS  = 180; 
const unsigned long RC_BACK_HOLD_MS = 70;  
const unsigned long RC_LEFT_HOLD_MS = 90;
const unsigned long RC_RIGHT_HOLD_MS = 90;

const double FB_ZERO_EPS   = 0.04;
const double TURN_ZERO_EPS  = 1.5;
const double RC_FB_SIGN = 1.0;

// Ramping registers
double currentSetpointOffset = 0.0;
double currentTurnOffset = 0.0;

// RC hold timers
unsigned long lastForwardActiveMs = 0;
unsigned long lastBackActiveMs = 0;
unsigned long lastLeftActiveMs = 0;
unsigned long lastRightActiveMs = 0;

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

double originalSetpoint = BALANCE_SETPOINT;
double setpoint = originalSetpoint;
double input, output;

// Your high-stability tuning values
double Kp = 22.0;       
double Kd = 1.8;      
double Ki = 135.0;    

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;

int ENA = 11; int IN1 = 7; int IN2 = 6;
int IN3 = 5;  int IN4 = 4; int ENB = 10;

LMotorController motorController(
  ENA, IN1, IN2, ENB, IN3, IN4,
  motorSpeedFactorLeft, motorSpeedFactorRight
);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

bool safetyFault = true;              
unsigned long uprightStartMs = 0;

void hardStop() {
  output = 0;
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);     analogWrite(ENB, 0);
}

void safeNativeDrive(int leftSpeed, int rightSpeed, int minAbs) {
  int realLeft = abs(leftSpeed);
  if (realLeft < minAbs) realLeft = minAbs;
  if (realLeft > 255) realLeft = 255;

  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, (int)(realLeft * motorSpeedFactorLeft));

  int realRight = abs(rightSpeed);
  if (realRight < minAbs) realRight = minAbs;
  if (realRight > 255) realRight = 255;

  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, (int)(realRight * motorSpeedFactorRight));
}

void setup() {
  Serial.begin(115200);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; 
  Wire.setWireTimeout(3000, true); 
#endif

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(RC_FORWARD_PIN, INPUT_PULLUP);
  pinMode(RC_BACK_PIN, INPUT_PULLUP);
  pinMode(RC_LEFT_PIN, INPUT_PULLUP);
  pinMode(RC_RIGHT_PIN, INPUT_PULLUP);

  hardStop();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0); mpu.setYGyroOffset(0); mpu.setZGyroOffset(0); mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(MANUAL);          
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    // ACTIVATE WATCHDOG: If the library loops infinitely, reboot in 120ms
    wdt_enable(WDTO_120MS); 
  }
}

void loop() {
  if (!dmpReady) return;

  // Reset the watchdog timer at the start of every iteration
  wdt_reset(); 

  if (mpuInterrupt || fifoCount >= packetSize) {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      hardStop();
      return;
    }
    else if (mpuIntStatus & 0x02) {
      unsigned long executionGuard = micros();
      while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
        if (micros() - executionGuard > 11000) { 
          mpu.resetFIFO();
          return;
        }
      }
      
      // Clear out the bus explicitly if a timeout was flag-triggered before reading bytes
      if (Wire.getWireTimeoutFlag()) {
        Wire.clearWireTimeoutFlag();
        mpu.resetFIFO();
        return;
      }

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      input = ypr[1] * 180 / M_PI + 180;

      if (safetyFault) {
        double absoluteError = fabs(input - originalSetpoint);
        if (absoluteError <= REARM_TILT_DEG) {
          if (uprightStartMs == 0) uprightStartMs = millis();
          if (millis() - uprightStartMs >= REARM_STABLE_MS) {
            safetyFault = false;
            pid.SetMode(AUTOMATIC);
            output = 0;
            currentSetpointOffset = 0.0;
            currentTurnOffset = 0.0;
          }
        } else {
          uprightStartMs = 0;
        }
        hardStop();
        return;
      }

      // ---------- RC READ & LOGIC ----------
      int forwardState = digitalRead(RC_FORWARD_PIN);
      int backState    = digitalRead(RC_BACK_PIN);
      int leftState    = digitalRead(RC_LEFT_PIN);
      int rightState   = digitalRead(RC_RIGHT_PIN);

      if (forwardState == LOW) lastForwardActiveMs = millis();
      if (backState == LOW)     lastBackActiveMs = millis();
      if (leftState == LOW)     lastLeftActiveMs = millis();
      if (rightState == LOW)    lastRightActiveMs = millis();

      bool forwardCmd = (millis() - lastForwardActiveMs) < RC_FWD_HOLD_MS;
      bool backCmd    = (millis() - lastBackActiveMs) < RC_BACK_HOLD_MS;
      bool leftCmd    = (millis() - lastLeftActiveMs) < RC_LEFT_HOLD_MS;
      bool rightCmd   = (millis() - lastRightActiveMs) < RC_RIGHT_HOLD_MS;

      double targetSetpointOffset = 0.0;
      double targetTurnOffset = 0.0;

      if (forwardCmd) {
        targetSetpointOffset = RC_FB_SIGN * MAX_FORWARD_ANGLE;
      }
      else if (backCmd) {
        targetSetpointOffset = -RC_FB_SIGN * MAX_BACK_ANGLE;
      }

      if (leftCmd) {
        targetTurnOffset = -MAX_TURN_SPEED;
      }
      else if (rightCmd) {
        targetTurnOffset = MAX_TURN_SPEED;
      }

      currentSetpointOffset += (targetSetpointOffset - currentSetpointOffset) * TILT_RAMP_FACTOR;
      currentTurnOffset     += (targetTurnOffset - currentTurnOffset) * TURN_RAMP_FACTOR;

      if (fabs(currentSetpointOffset) < FB_ZERO_EPS) currentSetpointOffset = 0.0;
      if (fabs(currentTurnOffset) < TURN_ZERO_EPS) currentTurnOffset = 0.0;

      setpoint = originalSetpoint + currentSetpointOffset;

      double activeAngleError = fabs(input - setpoint);
      if (activeAngleError > SAFETY_TILT_DEG || isnan(input)) {
        safetyFault = true;
        uprightStartMs = 0;
        currentSetpointOffset = 0.0;
        currentTurnOffset = 0.0;
        pid.SetMode(MANUAL);
        hardStop();
        return;
      }

      pid.Compute();

      int leftMotorOutput  = (int)output + (int)currentTurnOffset;
      int rightMotorOutput = (int)output - (int)currentTurnOffset;

      safeNativeDrive(leftMotorOutput, rightMotorOutput, MIN_ABS_SPEED);
    }
  }
}
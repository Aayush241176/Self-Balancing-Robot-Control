// Self Balancing Robot - Setpoint Calibration Sketch
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; 
uint8_t mpuIntStatus; 
uint8_t devStatus; 
uint16_t packetSize; 
uint16_t fifoCount; 
uint8_t fifoBuffer[64]; 

// Orientation vars
Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // [x, y, z] gravity vector
float ypr[3];           // [yaw, pitch, roll] container

double input;

// Variables for calculating a stable average
double totalInput = 0;
int sampleCount = 0;

volatile bool mpuInterrupt = false; 
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println(F("Initializing I2C devices..."));
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // If you have calibrated offsets, enter them here. Otherwise, leave at 0 for now.
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0); 

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino pin 2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    Serial.println(F("\n--- CALIBRATION READY ---"));
    Serial.println(F("1. Hold your robot perfectly vertical at its balance point."));
    Serial.println(F("2. Look at the 'Current Angle' values below."));
    Serial.println(F("3. Once the robot is steady, use the 'Rolling Average' as your originalSetpoint.\n"));
    delay(2000);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) return;

  // Wait for MPU interrupt or extra packet(s)
  while (!mpuInterrupt && fifoCount < packetSize) {
    // Motors are completely ignored here so the robot doesn't jump out of your hands
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // This is the exact pitch angle calculation from your original code
    input = ypr[1] * 180 / M_PI + 180;

    // Calculate a rolling average over 50 samples to filter out hand shakes
    totalInput += input;
    sampleCount++;
    
    if (sampleCount >= 50) {
      double rollingAverage = totalInput / 50;
      
      Serial.print(F("Current Angle: "));
      Serial.print(input);
      Serial.print(F("  |  [RECOMMENDED SETPOINT]: "));
      Serial.println(rollingAverage);
      
      // Reset averaging variables
      totalInput = 0;
      sampleCount = 0;
    }
  }
}
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define LED_PIN 13
bool blinkState = true;
Servo Servo1;
Servo Servo2; 
int Servo1Pos = 0;
int Servo2Pos = 0;
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
MPU6050 mpu
uint8_t mpuIntStatus; 
uint8_t devStatus; 
uint16_t packetSize;
uint16_t fifoCount; 
uint8_t fifoBuffer[64]; 
Quaternion q; 
VectorInt16 aa; 
VectorInt16 aaReal; 
VectorInt16 aaWorld; 
VectorFloat gravity; 
float ypr[3]; 
#define PITCH 1 
#define ROLL 
#define YAW 0 
// ================================================================
// === INITIAL SETUP ===
// ================================================================
void setup()
{
 Servo1.attach(10); 
 Servo2.attach(11); 
 delay(50);
 Servo1.write(0); 
 Servo2.write(60); 
 delay(500);
 Servo1.write(180);
 Servo2.write(120);
 delay(500);
 Servo1.write(0);
 Servo2.write(90);
 delay(500);#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
#endif
 Serial.begin(115200);
 while (!Serial); 
 Serial.println(F("Initializing I2C devices..."));
 mpu.initialize();
 Serial.println(F("Testing device connections..."));
 Serial.println(mpu.testConnection() ? F("MPU6050 connection 
successful") : F("MPU6050 connection failed"));
 
 Serial.println(F("Initializing DMP"));
devStatus = mpu.dmpInitialize();
 mpu.setXGyroOffset(118);
 mpu.setYGyroOffset(-44);
 mpu.setZGyroOffset(337);
 mpu.setXAccelOffset(-651);
 mpu.setYAccelOffset(670);
 mpu.setZAccelOffset(1895);
 if (devStatus == 0)
 {
 
 Serial.println(F("Enabling DMP"));
 mpu.setDMPEnabled(true);
 
 Serial.println(F("Enabling interrupt detection (Arduino external 
interrupt 0)"));
 mpuIntStatus = mpu.getIntStatus();
 packetSize = mpu.dmpGetFIFOPacketSize();
 }
 else
 {
 Serial.print(F("DMP Initialization failed code = "));
 Serial.println(devStatus);
 }
 pinMode(LED_PIN, OUTPUT);
} 
// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================
void loop(void){
 processAccelGyro();
} 
// ================================================================
// === PROCESS ACCEL/GYRO IF AVAILABLE ===
// ================================================================
void processAccelGyro()
{
 mpuIntStatus = mpu.getIntStatus();
 fifoCount = mpu.getFIFOCount();
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));
 return;
 }
 if (mpuIntStatus & 0x02) {
 if (fifoCount < packetSize)
 return; // fifoCount = mpu.getFIFOCount();
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 fifoCount -= packetSize;
 mpu.resetFIFO();
 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 mpuPitch = ypr[PITCH] * 180 / M_PI;
 mpuRoll = ypr[ROLL] * 180 / M_PI;
 mpuYaw = ypr[YAW] * 180 / M_PI;
 mpu.resetFIFO();
 blinkState = !blinkState;
 digitalWrite(LED_PIN, blinkState);
 mpu.resetFIFO();
 Servo1.write(-mpuPitch + 90);
 Servo2.write(mpuRoll + 90);
 delay(10);
 
 mpu.resetFIFO();
}
}

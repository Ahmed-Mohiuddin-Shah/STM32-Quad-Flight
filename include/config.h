#ifndef CONFIG_H
#define CONFIG_H

#define ENABLE_HWSERIAL2
#define ENABLE_HWSERIAL6

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <IBusBM.h>
#include "HardwareSerial.h"
#include "Servo.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DisplayController.h>
#include "TimerInterrupt_Generic.h"
#include "ISR_Timer_Generic.h"

#define TIMER_INTERVAL_MS 100
#define HW_TIMER_INTERVAL_MS 50

// Init STM32 timer TIM1
STM32Timer ITimer(TIM1);

// Init STM32_ISR_Timer
// Each STM32_ISR_Timer can service 16 different ISR-based timers
ISR_Timer STM32_ISR_Timer;

#define TIMER_INTERVAL_0_1S 100L

// Timer ID
#define TIMER_ID 1

// MPU6050 Pins
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// I2C 2 Pins for Debugging OLED
#define SDA2_PIN PB3
#define SCL2_PIN PB10

// OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

TwoWire Wire2(SDA2_PIN, SCL2_PIN);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

DisplayController displayController(&display);

// IBusBM Pins
// IBus Sensor
HardwareSerial Serial6(USART6);
IBusBM IBusSensor;
// IBus Servo
HardwareSerial Serial2(USART2);
IBusBM IBusServo;

// Servo motors pins
Servo motorLF;
Servo motorRF;
Servo motorLR;
Servo motorRR;

// Throttle Settings
float throttleIdle = 0.15;
float throttleGoverner = 0.7;

// Max attitude rate of change rates (degrees per second)
float maxRollRate = 30.0;
float maxPitchRate = 30.0;
float maxYawRate = 30.0;

// Desired Flight Controller Cycle time
// This is the number of times per second the flight controller will perform an adjustment loop (PID loop)
float targetCycleHz = 250.0;

// PID Controller Values
float pidRollKp = 0.00043714285;
float pidRollKi = 0.00255;
float pidRollKd = 0.00002571429;
float pidPitchKp = pidRollKp;
float pidPitchKi = pidRollKi;
float pidPitchKd = pidRollKd;
float pidYawKp = 0.001714287;
float pidYawKi = 0.003428571;
float pidYawKd = 0.0;

// float pidRollKp = 1.0;
// float pidRollKi = 0.0;
// float pidRollKd = 0.0;
// float pidPitchKp = 1.0;
// float pidPitchKi = 0.0;
// float pidPitchKd = 0.0;
// float pidYawKp = 1.0;
// float pidYawKi = 0.0;
// float pidYawKd = 0.0;

// runtime variables
float gyro_bias_x;
float gyro_bias_y;
float gyro_bias_z;

float cycleTimeSeconds = 1.0 / targetCycleHz;
float cycleTimeMicroseconds = cycleTimeSeconds * 1000000.0;
float maxThrottle = throttleGoverner;
float throttleRange = maxThrottle - throttleIdle;
float iLimit = 150.0; // PID I - term limiter.The applied I - term cannot exceed or go below(negative) this value.(safety mechanism to prevent excessive spooling of the motors)
bool lastMode = false;

// PID variables
float rollLastIntegral = 0.0;
float pitchLastIntegral = 0.0;
float yawLastIntegral = 0.0;
float rollLastError = 0.0;
float pitchLastError = 0.0;
float yawLastError = 0.0;

int loopBeginMicroseconds = 0;
int loopEndMicroseconds = 0;
int elapsedMicroseconds = 0;

int16_t gyro_x, gyro_y, gyro_z;
float float_gyro_x, float_gyro_y, float_gyro_z;

// rc input values
float rcRoll, rcPitch, rcYaw, rcThrottle;

float adjustedThrottle = 0;

float errorRateRoll = 0;
float errorRatePitch = 0;
float errorRateYaw = 0;

float rollP = 0;
float rollI = 0;
float rollD = 0;
float pidRoll = 0;

float pitchP = 0;
float pitchI = 0;
float pitchD = 0;
float pidPitch = 0;

float yawP = 0;
float yawI = 0;
float yawD = 0;
float pidYaw = 0;

float throttle1 = 0;
float throttle2 = 0;
float throttle3 = 0;
float throttle4 = 0;

#endif
// stm32f401ccu6
#include <Arduino.h>
#include <stdio.h>
#include <numeric>
#include "config.h"

void TimerHandler()
{
  STM32_ISR_Timer.run();
}

float normalize(int value, int min, int max)
{
  return (float)(value - min) / (max - min);
}

int convertToMicroseconds(float value, int min, int max)
{
  return map(constrain(value, 0, 1) * 1000, 0, 1000, min, max);
}

void setup()
{

  // Setup and calibrate the motors
  // Initialize servos
  motorLF.attach(PB13, 1000, 2000, 1500);
  motorRF.attach(PB14, 1000, 2000, 1500);
  motorLR.attach(PB15, 1000, 2000, 1500);
  motorRR.attach(PA8, 1000, 2000, 1500);
  // Initialize OLED
  delay(1000);
  displayController.begin();
  // Initialize Timer
  ITimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler);
  STM32_ISR_Timer.setInterval(TIMER_INTERVAL_0_1S, []()
                              { displayController.update(); });

  displayController.runStartupAnimation();
  delay(1000);

  displayController.showMessage("Calibrating Motors");
  motorLF.writeMicroseconds(2000);
  motorRF.writeMicroseconds(2000);
  motorLR.writeMicroseconds(2000);
  motorRR.writeMicroseconds(2000);
  delay(5000);
  motorLF.writeMicroseconds(1000);
  motorRF.writeMicroseconds(1000);
  motorLR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  displayController.showMessage("Motors Calibrated");
  delay(3000);

  // Blink the LED to show we are alive
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
  delay(500);
  digitalWrite(PC13, LOW);
  delay(500);
  digitalWrite(PC13, HIGH);
  delay(500);

  // Initialize MPU6050
  // wait 3 seconds for the MPU6050 to settle
  displayController.showMessage("Initializing MPU6050\nWaiting 3 seconds\nfor MPU to settle");
  delay(3000);
  Wire.begin();
  accelgyro.initialize();
  // set low pass filter to 5 (0-6)
  accelgyro.setDLPFMode(5);
  // set gyro scale to 1 (0-3)
  accelgyro.setFullScaleGyroRange(1);
  // set accel scale to 0 (0-3)
  accelgyro.setFullScaleAccelRange(0);
  accelgyro.setRate(4); // 200Hz
  accelgyro.setSleepEnabled(false);

  displayController.showMessage("MPU6050 Initialized\n" + String(accelgyro.testConnection() ? "Connection Successful" : "Connection Failed"));

  delay(1000);
  displayController.showMessage("Initializing IBus");
  // iBUS setup
  IBusServo.begin(Serial2);
  IBusSensor.begin(Serial6);

  delay(1000);
  IBusServo.loop();

  // check that flight mode is not on when we are first starting - it needs to be set into standby mode
  // this is a safety check. Prevents from the drone taking off (at least spinning props) as soon as power is plugged in
  while (IBusServo.readChannel(4) == 2000)
  {
    digitalWrite(PC13, HIGH);
    displayController.showMessage("Set Flight Mode to Standby\nYou are in flight mode\n!!!!!!!!\n" + String(IBusServo.readChannel(4)));
    delay(200);
    IBusServo.loop();
  }

  while (IBusServo.readChannel(4) == 0)
  {
    digitalWrite(PC13, HIGH);
    displayController.showMessage("RC Controller\nis off!\n!!!!!!!!\n" + String(IBusServo.readChannel(4)));
    delay(200);
    IBusServo.loop();
  }

  displayController.showMessage("Flight Mode Set");
  digitalWrite(PC13, LOW);

  // adding 2 sensors
  IBusSensor.addSensor(IBUSS_EXTV);
  IBusSensor.addSensor(IBUSS_TEMP);

  // Measuring Gyro Bias
  displayController.showMessage("Measuring Gyro Bias");
  std::vector<float> gxs, gys, gzs;
  unsigned long started_at_ticks_ms = millis();
  while ((millis() - started_at_ticks_ms) / 1000.0 < 3.0)
  {
    accelgyro.getRotation(&gyro_x, &gyro_y, &gyro_z);
    gxs.push_back(gyro_x / 65.5);
    gys.push_back(gyro_y / 65.5);
    gzs.push_back((gyro_z / 65.5) * -1); // Adjust for mounting orientation
    delay(25);
  }
  gyro_bias_x = std::accumulate(gxs.begin(), gxs.end(), 0.0) / gxs.size();
  gyro_bias_y = std::accumulate(gys.begin(), gys.end(), 0.0) / gys.size();
  gyro_bias_z = std::accumulate(gzs.begin(), gzs.end(), 0.0) / gzs.size();

  displayController.showMessage("Gyro Bias:\nx: " + String(gyro_bias_x) + "\ny: " + String(gyro_bias_y) + "\nz: " + String(gyro_bias_z));

  delay(3000);

  displayController.showMessage("Ready");
  delay(1000);
  displayController.showMessage("disabling Interrupts");
  delay(1000);
  // disable all timer interrupts
  STM32_ISR_Timer.disableAll();
  ITimer.detachInterrupt();

  displayController.showMessage("Starting Flight Controller");
  digitalWrite(PC13, HIGH);
  delay(1000);
}

void standByMode() {

  displayController.showMessage("Standby Mode");

  motorLF.writeMicroseconds(1000);
  motorRF.writeMicroseconds(1000);
  motorLR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);

  // reset PIDs
  rollLastIntegral = 0.0;
  pitchLastIntegral = 0.0;
  yawLastIntegral = 0.0;
  rollLastError = 0.0;
  pitchLastError = 0.0;
  yawLastError = 0.0;

  // set last mode
  lastMode = false;

  return;
}

void flightMode() {

  // calculate adjusted throttle
  adjustedThrottle = (rcThrottle * throttleRange) + throttleIdle;

  // calculate errors - diff between the actual rates and the desired rates
  // "error" is calculated as setpoint(the goal) - actual
  errorRateRoll = (rcRoll * maxRollRate) - (float_gyro_x);
  errorRatePitch = (-rcPitch * maxPitchRate) - (float_gyro_y);
  errorRateYaw = (rcYaw * maxYawRate) - (float_gyro_z);

  // displayController.showMessage("Error Roll: " + String(errorRateRoll) + "\nError Pitch: " + String(errorRatePitch) + "\nError Yaw: " + String(errorRateYaw));

  // calculate PIDs
  // P - Proportional - the error is multiplied by a constant
  // I - Integral - the sum of all errors is multiplied by a constant
  // D - Derivative - the rate of change of the error is multiplied by a constant

  // roll pid
  rollP = pidRollKp * errorRateRoll;
  rollI = rollLastIntegral + (pidRollKi * errorRateRoll * cycleTimeSeconds);
  rollI = constrain(rollI, -iLimit, iLimit);
  rollD = (pidRollKd * (errorRateRoll - rollLastError)) / cycleTimeSeconds;
  pidRoll = rollP + rollI + rollD;

  // pitch pid
  pitchP = pidPitchKp * errorRatePitch;
  pitchI = pitchLastIntegral + (pidPitchKi * errorRatePitch * cycleTimeSeconds);
  pitchI = constrain(pitchI, -iLimit, iLimit);
  pitchD = (pidPitchKd * (errorRatePitch - pitchLastError)) / cycleTimeSeconds;
  pidPitch = pitchP + pitchI + pitchD;

  // yaw pid
  yawP = pidYawKp * errorRateYaw;
  yawI = yawLastIntegral + (pidYawKi * errorRateYaw * cycleTimeSeconds);
  yawI = constrain(yawI, -iLimit, iLimit);
  yawD = (pidYawKd * (errorRateYaw - yawLastError)) / cycleTimeSeconds;
  pidYaw = yawP + yawI + yawD;

  throttle1 = adjustedThrottle + pidPitch + pidRoll - pidYaw;
  throttle2 = adjustedThrottle + pidPitch - pidRoll + pidYaw;
  throttle3 = adjustedThrottle - pidPitch + pidRoll + pidYaw;
  throttle4 = adjustedThrottle - pidPitch - pidRoll - pidYaw;

  throttle1 = convertToMicroseconds(throttle1, 1000, 2000);
  throttle2 = convertToMicroseconds(throttle2, 1000, 2000);
  throttle3 = convertToMicroseconds(throttle3, 1000, 2000);
  throttle4 = convertToMicroseconds(throttle4, 1000, 2000);

  // set motor outputs
  motorLF.writeMicroseconds(throttle1);
  motorRF.writeMicroseconds(throttle2);
  motorLR.writeMicroseconds(throttle3);
  motorRR.writeMicroseconds(throttle4);

  // displayController.showMessage("Throttle: " + String(adjustedThrottle) + "\nRC Throttle: " + String(rcThrottle) +
  //                               "\nThrottle1: " + String(throttle1) + "\nThrottle2: " + String(throttle2) + "\nThrottle3: " + String(throttle3) + "\nThrottle4: " + String(throttle4));

  // set last errors
  rollLastError = errorRateRoll;
  pitchLastError = errorRatePitch;
  yawLastError = errorRateYaw;
  rollLastIntegral = rollI;
  pitchLastIntegral = pitchI;
  yawLastIntegral = yawI;

  // display values
  // displayController.showMessage("\nThrottle: " + String(adjustedThrottle) + "\nRC Throttle: " + String(rcThrottle) +
  // "\nThrottle1: " + String(throttle1) + "\nThrottle2: " + String(throttle2) + "\nThrottle3: " + String(throttle3) + "\nThrottle4: " + String(throttle4));

  return;
}

void mainLoop()
{

  // Capture Raw IMU Data
  accelgyro.getRotation(&gyro_x, &gyro_y, &gyro_z);

  // convert gyro values to degrees per second
  float_gyro_x = (float)(gyro_x) / 65.5;
  float_gyro_y = (float)(gyro_y) / 65.5;
  float_gyro_z = ((float)(gyro_z) / 65.5); // Adjust for mounting orientation

  // if readings +/- 1, set to 0
  if (float_gyro_x < 2 && float_gyro_x > -2)
  {
    float_gyro_x = 0;
  }
  if (float_gyro_y < 2 && float_gyro_y > -2)
  {
    float_gyro_y = 0;
  }
  if (float_gyro_z < 2 && float_gyro_z > -2)
  {
    float_gyro_z = 0;
  }
  

  displayController.showMessage("Gyro X: " + String(float_gyro_x) + "\nGyro Y: " + String(float_gyro_y) + "\nGyro Z: " + String(float_gyro_z));

  // Normalize all RC inputs to 0-1
  rcThrottle = normalize(IBusServo.readChannel(2), 1000, 2000);

  // normalize all RC inputs to -1.0 - 1.0
  rcRoll = normalize(IBusServo.readChannel(1), 1000, 2000) * 2.0 - 1.0;
  rcPitch = normalize(IBusServo.readChannel(0), 1000, 2000) * 2.0 - 1.0;
  rcYaw = normalize(IBusServo.readChannel(3), 1000, 2000) * 2.0 - 1.0;

  // displayController.showMessage("Throttle: " + String(rcThrottle) + "\nRoll: " + String(rcRoll) + "\nPitch: " + String(rcPitch) + "\nYaw: " + String(rcYaw));

  // ADJUST MOTOR OUTPUTS !
  // based on channel 5. Channel 5 I have assigned to the switch that determines flight mode(standby / flight)

  // if the switch is in standby mode, then the motors should be off
  switch (IBusServo.readChannel(4))
  {
  case 2000:
    // if last mode was standby(we JUST were turned onto flight mode), perform a check that the throttle isn't high. This is a safety mechanism
    // this prevents an accident where the flight mode switch is turned on but the throttle position is high, which would immediately apply heavy throttle to each motor, shooting it into the air.
    if (lastMode == false)
    {
      if (rcThrottle > 0.05)
      {
        displayController.showMessage("Throttle is too high\nSet throttle to idle\n!!!!!!!!");
        motorLF.writeMicroseconds(1000);
        motorRF.writeMicroseconds(1000);
        motorLR.writeMicroseconds(1000);
        motorRR.writeMicroseconds(1000);
        break;
      }
    }

    // set last mode
    lastMode = true;
    flightMode();
    break;
  case 1000:
  case 1500:
    standByMode();
    break;
  default:
    standByMode();
  }
}

void loop()
{

  loopBeginMicroseconds = micros();

  IBusSensor.loop();          // process sensor requests
  IBusServo.loop();           // process servo requests
  displayController.update(); // process display requests

  mainLoop();

  loopEndMicroseconds = micros();

  elapsedMicroseconds = loopEndMicroseconds - loopBeginMicroseconds;
  if (elapsedMicroseconds < cycleTimeMicroseconds)
  {
    delayMicroseconds(cycleTimeMicroseconds - elapsedMicroseconds);
  }
}

// This example shows how to automatically measure the current
// sense offsets at startup and load them into the Motoron so that the
// processed current measurements are more accurate.

#include <Motoron.h>

MotoronI2C mc;

// ADC reference voltage: change to 3300 if using a 3.3 V Arduino.
const uint16_t referenceMv = 5000;

// Minimum allowed VIN voltage.  You can raise this to be closer
// to your power supply's expected voltage.
const uint16_t minVinVoltageMv = 4500;

void calibrateCurrent()
{
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);

  const uint8_t desiredSampleCount = 32;
  uint8_t sampleCount = 0;
  uint16_t totals[2] = { 0, 0 };

  uint16_t lastTimeConditionsNotMet = millis();
  while (1)
  {
    const uint16_t statusMask = (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING) |
      (1 << MOTORON_STATUS_FLAG_NO_POWER) |
      (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED) |
      (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING);
    const uint16_t statusDesired = (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED);

    if ((mc.getStatusFlags() & statusMask) != statusDesired ||
      (mc.getVinVoltageMv(referenceMv) < minVinVoltageMv))
    {
      lastTimeConditionsNotMet = millis();
      sampleCount = 0;
      memset(totals, 0, sizeof(totals));
      totals[0] = totals[1] = 0;
    }

    if ((uint16_t)(millis() - lastTimeConditionsNotMet) > 20)
    {
      totals[0] += mc.getCurrentSense(1);
      totals[1] += mc.getCurrentSense(2);
      if (sampleCount++ == desiredSampleCount) { break; }
    }
  }

  mc.setCurrentSenseOffset(1, (totals[0] + desiredSampleCount / 2) / desiredSampleCount);
  mc.setCurrentSenseOffset(2, (totals[1] + desiredSampleCount / 2) / desiredSampleCount);

  Serial.print(F("Current sense offsets: "));
  Serial.print(mc.getCurrentSenseOffset(1));
  Serial.print(' ');
  Serial.println(mc.getCurrentSenseOffset(2));
}

void setup()
{
  while (!Serial);
  Wire.begin();

  mc.reinitialize();
  mc.clearResetFlag();

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();

  // Configure motor 1
  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 200);

  // Configure motor 2
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 200);

  calibrateCurrent();
}

void loop()
{
  mc.setSpeed(1, 200);
  delay(1000);
  Serial.print(F("Motor 1 current: "));
  Serial.println(mc.getCurrentSenseProcessed(1));
  mc.setSpeed(1, 0);
  delay(1000);

  mc.setSpeed(2, 00);
  delay(1000);
  Serial.print(F("Motor 2 current: "));
  Serial.println(mc.getCurrentSenseProcessed(2));
  mc.setSpeed(2, 0);
  delay(1000);
}

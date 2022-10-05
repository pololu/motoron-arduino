// This example for the Motoron M2S controllers shows how to
// automatically measure the current sense offsets at startup
// and load them into the Motoron so that the processed current
// measurements are more accurate.
//
// It also uses those current sense offsets to help set current
// limits.

#include <Motoron.h>

MotoronI2C mc;

// ADC reference voltage: change to 3300 if using a 3.3 V Arduino.
const uint16_t referenceMv = 5000;

// Specifies what type of Motoron you are using, which is needed
// for converting current sense readings to milliamps.
const MotoronCurrentSenseType type = MotoronCurrentSenseType::Motoron18v18;

// Minimum allowed VIN voltage.  You can raise this to be closer
// to your power supply's expected voltage.
const uint16_t minVinVoltageMv = 6500;

const uint16_t units = mc.currentSenseUnitsMilliamps(type, referenceMv);

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
      totals[0] = totals[1] = 0;
    }

    if ((uint16_t)(millis() - lastTimeConditionsNotMet) > 20)
    {
      totals[0] += mc.getCurrentSenseRaw(1);
      totals[1] += mc.getCurrentSenseRaw(2);
      if (++sampleCount == desiredSampleCount) { break; }
    }
  }

  mc.setCurrentSenseOffset(1, (totals[0] + desiredSampleCount / 2) / desiredSampleCount);
  mc.setCurrentSenseOffset(2, (totals[1] + desiredSampleCount / 2) / desiredSampleCount);
  mc.setCurrentSenseMinimumDivisor(1, 100);
  mc.setCurrentSenseMinimumDivisor(2, 100);

  Serial.print(F("Current sense offsets: "));
  Serial.print(mc.getCurrentSenseOffset(1));
  Serial.print(' ');
  Serial.println(mc.getCurrentSenseOffset(2));
}

void setup()
{
  // For boards with native USB, this line waits for the serial
  // monitor to be open.
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

  // Set current limits using the offsets we just measured.
  // The first argument to calculateCurrentLimit is a
  // current limit in milliamps.
  mc.setCurrentLimit(1, mc.calculateCurrentLimit(10000,
    type, referenceMv, mc.getCurrentSenseOffset(1)));
  mc.setCurrentLimit(2, mc.calculateCurrentLimit(10000,
    type, referenceMv, mc.getCurrentSenseOffset(2)));
}

void printCurrent(uint16_t processed)
{
  Serial.print(processed);
  Serial.print(F(" = "));
  uint32_t ma = (uint32_t)processed * units;
  Serial.print(ma);
  Serial.println(F(" mA"));
}

void loop()
{
  mc.setSpeed(1, 200);
  delay(1000);
  Serial.print(F("Motor 1 current: "));
  printCurrent(mc.getCurrentSenseProcessed(1));
  mc.setSpeed(1, 0);
  delay(1000);

  mc.setSpeed(2, 200);
  delay(1000);
  Serial.print(F("Motor 2 current: "));
  printCurrent(mc.getCurrentSenseProcessed(2));
  mc.setSpeed(2, 0);
  delay(1000);
}

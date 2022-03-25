// This example shows a simple way to control the
// Motoron Motor Controller.
//
// The motors will stop but automatically recover if:
// - Motor power (VIN) is interrupted, or
// - A temporary motor fault occurs, or
// - A command timeout occurs.
//
// The motors will stop until you power cycle or reset your
// Arduino if:
// - The Motoron experiences a reset.
//
// If a latched motor fault occurs, the motors
// experiencing the fault will stop until you power cycle motor
// power (VIN) or cause the motors to coast.

#include <Motoron.h>

MotoronI2C mc;

void setup()
{
  Wire.begin();

  // Reset the controller to its default settings, then disable
  // CRC.  The bytes for each of these commands are shown here
  // in case you want to implement them on your own without
  // using the library.
  mc.reinitialize();    // Bytes: 0x96 0x74
  mc.disableCrc();      // Bytes: 0x8B 0x04 0x7B 0x43

  // Clear the reset flag, which is set after the controller
  // reinitializes and counts as an error.
  mc.clearResetFlag();  // Bytes: 0xA9 0x00 0x04

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();

  // Configure motor 1
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);

  // Configure motor 2
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);

  // Configure motor 3
  mc.setMaxAcceleration(3, 80);
  mc.setMaxDeceleration(3, 300);
}

void loop()
{
  if (millis() & 2048)
  {
    mc.setSpeed(1, 800);
  }
  else
  {
    mc.setSpeed(1, -800);
  }

  mc.setSpeed(2, 100);
  mc.setSpeed(3, -100);
}

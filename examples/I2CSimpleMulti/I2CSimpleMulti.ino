// This example shows a simple way to control multiple
// Motoron Motor Controllers.
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

// This code creates an object for each Motoron controller.
// The number passed as the first argument to each constructor
// below should be the 7-bit I2C address of the controller.
//
// You should use the SetI2cAddresses sketch to
// assign a unique I2C address to each Motoron, and then
// modify the list below to match your setup.
MotoronI2C mc1(17);
MotoronI2C mc2(18);
MotoronI2C mc3(19);
MotoronI2C mc4(20);

// You can call functions directly on each of the objects
// created above (mc1, mc2, etc.) but if you want to write
// reusable code, the function below shows how to do that
// using references, a feature of the C++ language.
void setupMotoron(MotoronI2C & mc)
{
  mc.reinitialize();
  mc.disableCrc();

  // Clear the reset flag, which is set after the controller
  // reinitializes and counts as an error.
  mc.clearResetFlag();

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();
}

void setup()
{
  Wire.begin();

  setupMotoron(mc1);
  setupMotoron(mc2);
  setupMotoron(mc3);
  setupMotoron(mc4);

  mc1.setMaxAcceleration(1, 80);
  mc1.setMaxDeceleration(1, 300);

  mc1.setMaxAcceleration(2, 80);
  mc1.setMaxDeceleration(2, 300);

  mc1.setMaxAcceleration(3, 80);
  mc1.setMaxDeceleration(3, 300);

  mc2.setMaxAcceleration(1, 80);
  mc2.setMaxDeceleration(1, 300);

  mc2.setMaxAcceleration(2, 80);
  mc2.setMaxDeceleration(2, 300);

  mc2.setMaxAcceleration(3, 80);
  mc2.setMaxDeceleration(3, 300);

  mc3.setMaxAcceleration(1, 80);
  mc3.setMaxDeceleration(1, 300);

  mc3.setMaxAcceleration(2, 80);
  mc3.setMaxDeceleration(2, 300);

  mc3.setMaxAcceleration(3, 80);
  mc3.setMaxDeceleration(3, 300);

  mc4.setMaxAcceleration(1, 80);
  mc4.setMaxDeceleration(1, 300);

  mc4.setMaxAcceleration(2, 80);
  mc4.setMaxDeceleration(2, 300);

  mc4.setMaxAcceleration(3, 80);
  mc4.setMaxDeceleration(3, 300);
}

void loop()
{
  int16_t changingSpeed = (millis() & 2048) ? 800 : -800;

  mc1.setSpeed(1, changingSpeed);
  mc1.setSpeed(2, 100);
  mc1.setSpeed(3, -100);

  mc2.setSpeed(1, 100);
  mc2.setSpeed(2, changingSpeed);
  mc2.setSpeed(3, -100);

  mc3.setSpeed(1, 100);
  mc3.setSpeed(2, -100);
  mc3.setSpeed(3, changingSpeed);

  mc4.setSpeed(1, changingSpeed);
  mc4.setSpeed(2, -changingSpeed);
  mc4.setSpeed(3, changingSpeed);
}

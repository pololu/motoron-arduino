// This example shows how to control a Motoron Motor Controller using its
// UART serial interface if you want your system to just keep working,
// ignoring or automatically recovering from errors as much as
// possible.
//
// The motors will stop but automatically recover if:
// - Motor power (VIN) is interrupted
// - A motor fault occurs
// - The Motoron experiences a reset
// - A command timeout occurs

#include <Motoron.h>

// On boards with a hardware serial port available for use, use
// that port to communicate with the Motoron. For other boards,
// create a SoftwareSerial object using pin 10 to receive (RX)
// and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define mcSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial mcSerial(10, 11);
#endif

MotoronSerial mc;

uint16_t lastTimeMotorsInit;

void motorsInit()
{
  mc.clearResetFlag();

  // By default, the Motoron is configured to stop the motors if
  // it does not get a motor control command for 1500 ms.  You
  // can uncomment a line below to adjust this time or disable
  // the timeout feature.
  // mc.setCommandTimeoutMilliseconds(1000);
  // mc.disableCommandTimeout();

  // Configure motor 1
  mc.setMaxAcceleration(1, 70);
  mc.setMaxDeceleration(1, 150);

  // Configure motor 2
  mc.setMaxAcceleration(2, 100);
  mc.setMaxDeceleration(2, 150);

  // Configure motor 3
  mc.setMaxAcceleration(3, 40);
  mc.setMaxDeceleration(3, 150);

  mc.clearMotorFaultUnconditional();

  lastTimeMotorsInit = millis();
}

void setup()
{
  mcSerial.begin(115200);
  mcSerial.setTimeout(20);
  mc.setPort(&mcSerial);

  mc.reinitialize();
  motorsInit();
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

  // Every 2 seconds, run motorsInit to restart the motors
  // in case anything has caused them to shut down.
  if ((uint16_t)(millis() - lastTimeMotorsInit) > 2000)
  {
    motorsInit();
  }
}

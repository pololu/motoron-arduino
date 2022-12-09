// This example shows how to control a Motoron Motor Controller using its
// UART serial interface if you want to shut down the motors whenever any
// problems are detected.
//
// The motors will stop until you power cycle or reset your
// Arduino if:
// - Motor power (VIN) is interrupted
// - A motor fault occurs
// - A protocol or CRC error occurs
// - A command timeout occurs
// - The Motoron experiences a reset
//
// Note: If you want to be even more careful, you might consider
// reading back all of the settings you care about and verifying
// they are correct at the end of loop().
//
// Note: If your Motoron has fewer than three motor channels,
// you should remove the commands in loop() and setup() that
// operate on the motors your controller does not have.
// Otherwise, those commands will cause a protocol error.

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

// ADC reference voltage: change to 3300 if using a 3.3 V Arduino.
const uint16_t referenceMv = 5000;

// Minimum allowed VIN voltage.  You can raise this to be closer
// to your power supply's expected voltage.
const uint16_t minVinVoltageMv = 4500;

// Define which status flags the Motoron should treat as errors.
const uint16_t errorMask =
    (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR) |
    (1 << MOTORON_STATUS_FLAG_CRC_ERROR) |
    (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED) |
    (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED) |
    (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED) |
    (1 << MOTORON_STATUS_FLAG_UART_ERROR) |
    (1 << MOTORON_STATUS_FLAG_RESET) |
    (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT);

void setup()
{
  Serial.begin(9600);
  mcSerial.begin(115200);
  mcSerial.setTimeout(20);
  mc.setPort(&mcSerial);

  mc.reinitialize();
  mc.clearResetFlag();

  // Configure the Motoron to coast the motors while obeying
  // decleration limits when there is an error (the default).
  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_COAST);

  mc.setErrorMask(errorMask);

  // Use a short command timeout of 100 ms: the Motoron will
  // stop the motors if it does not get a command for 100 ms.
  mc.setCommandTimeoutMilliseconds(100);

  // Configure motor 1
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);

  // Configure motor 2
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);

  // Configure motor 3
  mc.setMaxAcceleration(3, 80);
  mc.setMaxDeceleration(3, 300);

  // Depending on what was happening before this sketch started,
  // the motors will either be stopped or decelerating.
  // This loop waits for them to stop so that when the rest of
  // the code starts running, it will run from
  // a more predictable starting point.  This is optional.
  while (mc.getMotorDrivingFlag());

  mc.clearMotorFault();
}

void checkCommunicationError(uint8_t errorCode)
{
  if (errorCode)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("Communication error: "));
      Serial.println(errorCode);
      delay(1000);
    }
  }
}

void checkControllerError(uint16_t status)
{
  if (status & errorMask)
  {
    while (1)
    {
      // One of the error flags is set.  The Motoron should
      // already be stopping the motors.  We report the issue to
      // the user and send reset commands to be extra careful.
      mc.reset();
      Serial.print(F("Controller error: 0x"));
      Serial.println(status, HEX);
      delay(1000);
    }
  }
}

void checkVinVoltage(uint16_t voltageMv)
{
  if (voltageMv < minVinVoltageMv)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("VIN voltage too low: "));
      Serial.println(voltageMv);
      delay(1000);
    }
  }
}

void checkForProblems()
{
  uint16_t status = mc.getStatusFlags();
  checkCommunicationError(mc.getLastError());
  checkControllerError(status);

  uint32_t voltageMv = mc.getVinVoltageMv(referenceMv);
  checkCommunicationError(mc.getLastError());
  checkVinVoltage(voltageMv);
}

void loop()
{
  checkForProblems();

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

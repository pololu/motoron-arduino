// This example shows how to control multiple Motoron Motor Controllers in a
// half-duplex RS-485 network while checking for errors.
//
// This is similar to SerialCarefulExample but it uses the
// "Multi-device write" command to efficiently send speeds to multiple Motorons
// and it uses the "Multi-device error check" command to efficiently check for
// errors on multiple Motorons.
//
// The error checking code only works if each addressed Motoron can see the
// repsonses sent by the other Motorons (e.g. they are in a half-duplex RS-485
// network).
//
// This code assumes you have configured the Motorons to send 7-bit responses,
// which is important because it prevents a Motoron from interpreting a response
// from another Motoron as a command.
//
// You will need to change the following settings in this code to match your
// hardware configuration: DE_PIN, RE_PIN, startingDeviceNumber, deviceCount,
// motorsPerDevice.

#include <Motoron.h>
#include "SerialWithDE.h"

// Define the pins that are used to control the RS-485 driver and receiver.
// These can be the same pin or they can be 0xFF to disable the feature.
#define DE_PIN 9
#define RE_PIN 10

// Define the range of Motoron device numbers to control.
// (Note that this code does send some compact protocol commands which will
// affect all Motorons regardless of their device number.)
const uint16_t startingDeviceNumber = 17;
const uint16_t deviceCount = 3;

// Define the number of motor channels per Motoron.
// It is OK if some of the Motorons in the system have fewer channels than this.
const uint16_t motorsPerDevice = 2;

#ifdef SERIAL_PORT_HARDWARE_OPEN
#define mcSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial mcSerial(10, 11);
#endif

SerialWithDE mcSerialWithDE(&mcSerial, DE_PIN, RE_PIN);

MotoronSerial mc;

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

uint8_t buffer[deviceCount * motorsPerDevice * 2];

void setSpeedInBuffer(uint8_t motor, int16_t speed)
{
  if (motor >= deviceCount * motorsPerDevice) { return; }
  buffer[motor * 2 + 0] = speed & 0x7F;
  buffer[motor * 2 + 1] = (speed >> 7) & 0x7F;
}

void setup()
{
  Serial.begin(9600);

  mcSerial.begin(115200);
  mcSerial.setTimeout(20);
  mcSerialWithDE.begin();

  mc.setPort(&mcSerialWithDE);
  mc.expect7BitResponses();
  // mc.use14BitDeviceNumber();

  // Reinitialize all the Motorons at once using the compact protocol.
  mc.reinitialize();
  mc.clearResetFlag();
  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_COAST);
  mc.setErrorMask(errorMask);

  for (uint8_t motor = 1; motor <= motorsPerDevice; motor++)
  {
    mc.setMaxAcceleration(motor, 100);
    mc.setMaxDeceleration(motor, 100);
  }

  // Device-specific initializations (optional).
  mc.setDeviceNumber(startingDeviceNumber);
  mc.setMaxAcceleration(1, 800);
  mc.setMaxDeceleration(1, 800);

  // Go back to using the compact protocol.
  mc.setDeviceNumber(0xFFFF);
}

void handleError(uint16_t deviceNumber)
{
  // A device has an error or cannot communicate.  Shut down all motors.
  mc.coastNow();

  // Try to get more detailed info from the device with the problem.
  mc.setDeviceNumber(deviceNumber);
  uint16_t statusFlags = mc.getStatusFlags();
  uint8_t error = mc.getLastError();
  mc.setDeviceNumber(0xFFFF);
  if (error)
  {
    while (1)
    {
      Serial.print(F("Failed to get status flags from device "));
      Serial.print(deviceNumber);
      Serial.print(F(": error "));
      Serial.println(error);
      mc.coastNow();
      delay(1000);
    }
  }
  else
  {
    while (1)
    {
      mc.coastNow();
      Serial.print(F("Error from device "));
      Serial.print(deviceNumber);
      Serial.print(F(": status 0x"));
      Serial.println(statusFlags);
      delay(1000);
    }
  }
}

void loop()
{
  // Check for errors on all the Motorons.
  uint16_t errorIndex = mc.multiDeviceErrorCheck(startingDeviceNumber, deviceCount);
  if (errorIndex != deviceCount)
  {
    handleError(startingDeviceNumber + errorIndex);
  }

  // Calculate motor speeds that will drive each motor forward briefly and
  // repeat every 4 seconds.
  for (uint16_t i = 0; i < motorsPerDevice * deviceCount; i++)
  {
    uint16_t phase = (millis() - 512 * i) % 4096;
    if (phase <= 500)
    {
      setSpeedInBuffer(i, 400);
    }
    else
    {
      setSpeedInBuffer(i, 0);
    }
  }

  // Send the motor speeds.
  mc.multiDeviceWrite(startingDeviceNumber, deviceCount, motorsPerDevice * 2,
    MOTORON_CMD_SET_ALL_SPEEDS, buffer);
}

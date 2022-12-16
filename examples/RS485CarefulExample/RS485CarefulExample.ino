#include <Motoron.h>
#include "SerialWithDE.h"

#define DE_PIN 9
#define RE_PIN 10

const uint16_t startingDeviceNumber = 17;
const uint16_t deviceCount = 3;
const uint16_t motorsPerDevice = 2;

SerialWithDE mcSerial(&SERIAL_PORT_HARDWARE_OPEN, DE_PIN, RE_PIN);

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
  mcSerial.begin(115200);
  mcSerial.setTimeout(20);

  mc.setPort(&mcSerial);
  mc.expect7BitResponses();

  // Reinitialize all the Motorons at once using the compact protocol.
  mc.reinitialize();
  mc.clearResetFlag();
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

void loop()
{
  uint16_t flags = mc.getStatusFlags();
  if (mc.getLastError())
  {
    Serial.print(F("Error: "));
    Serial.println(mc.getLastError());
  }
  else
  {
    Serial.println(flags, HEX);
  }

  // Drive each motor forward briefly and repeat every 4 seconds.
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

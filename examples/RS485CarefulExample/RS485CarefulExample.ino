#include <Motoron.h>
#include "SerialWithDE.h"

#define DE_PIN 9
#define RE_PIN 10

uint16_t startingDeviceNumber = 17;
uint16_t deviceCount = 3;
uint16_t motorsPerDevice = 2;

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

void setup() {
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

  // Do some device-specific initializations (optional).
  mc.setDeviceNumber(startingDeviceNumber);
  mc.setMaxAcceleration(1, 400);
  mc.setMaxDeceleration(1, 400);

  // Go back to using the compact protocol.
  mc.setDeviceNumber(0xFFFF);
}

void loop() {

  delay(100);  // tmphax
  while (mcSerial.available()) { mcSerial.read(); }  // tmphax
  uint16_t flags = mc.getStatusFlags();
  if (mc.getLastError())
  {
    Serial.print(F("Error: "));
    Serial.println(mc.getLastError());
  }
  else
  {
    Serial.println(flags);
  }
  delay(100);

  if (millis() & 2048)
  {
    mc.setSpeed(1, 800);
  }
  else
  {
    mc.setSpeed(1, -800);
  }
}

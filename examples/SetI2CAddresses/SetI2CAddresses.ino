// This example sketch provides an interactive utility you can
// use to set the I2C addresses for a single Motoron controller
// or a stack of controllers.
//
// After uploading this sketch to your Arduino, open the
// Serial Monitor (from the Tools menu of the Arduino IDE).
//
// To scan your I2C bus for devices, send a line starting with
// "s".
//
// To change the I2C address of a Motoron, short the Motoron's
// JMP1 pin to GND, then send a line starting with "w", followed
// by the address you want to assign as a decimal number.  For
// example, "w17" sets the address of the Motoron to decimal 17.
//
// Alternatively, you can send a line that just contains "w"
// by itself in order to have the sketch automatically pick a
// number for you.
//
// After you are done changing the address of a Motoron, you
// should stop shorting its JMP1 pin to GND.
//
// The commands in this sketch use the I2C general call address
// (0), so they might interfere with other devices that use that
// address, and they will not work if you previously disabled
// the general call address on a Motoron and have not reset the
// Motoron since then.
//
// The new I2C address will not actually be used until you reset
// the Motoron, power cycle it, or use the "u" command.
//
// This sketch expects each command to be terminated with a
// line-ending character ('\r' or '\n').  The serial monitor
// automatically sends a line-ending character when you click
// the Send button or press Enter.  Do not send quote characters.

#include <Motoron.h>

// Make a MotoronI2C object configured to use the general call
// address (0).
MotoronI2C mc(0);

uint8_t nextAddress = 17;

uint8_t lineSize;
char lineBuffer[40];

#ifdef __SAM3X8E__
// Arduino Due uses Wire1
#define WIRE Wire1
#else
#define WIRE Wire
#endif

void setup()
{
  Serial.begin(9600);
  WIRE.begin();
  mc.setBus(&WIRE);
}

uint8_t getNextAddress(uint8_t address)
{
  address = address + 1;
  if (address == 0 || address >= 128) { address = 1; }
  return address;
}

void scanForDevices()
{
  Serial.println(F("Scanning for I2C devices..."));
  for (uint8_t i = 0; i < 128; i++)
  {
    WIRE.beginTransmission(i);
    uint8_t error = WIRE.endTransmission();
    switch (error)
    {
    case 0:
      Serial.print(F("Found device at address "));
      Serial.println(i);
      break;
    case 2:
      // No device at address i: we received a NACK when
      // sending the address.
      break;
    default:
      Serial.print(F("Unexpected result at address "));
      Serial.print(i);
      Serial.print(F(": "));
      Serial.println(error);
      break;
    }
  }
  Serial.println(F("Done."));
}

void writeAddress()
{
  bool desiredAddressSpecified = false;
  uint8_t desiredAddress = nextAddress;
  if (lineBuffer[1])
  {
    desiredAddress = strtoul(lineBuffer + 1, NULL, 10) & 127;
    desiredAddressSpecified = true;
  }

  // Make sure there is not already a device on the bus using
  // the desired address.
  while (1)
  {
    WIRE.beginTransmission(desiredAddress);
    uint8_t error = WIRE.endTransmission();
    if (error == 2) { break; }

    Serial.print(F("Found a device at address "));
    Serial.print(desiredAddress);
    Serial.println('.');
    if (desiredAddressSpecified)
    {
      // The user specified what address they want to use,
      // so just return.
      return;
    }
    else
    {
      // Try the next higher address.
      desiredAddress = nextAddress(desiredAddress);
      continue;
    }
  }

  mc.enableCrc();

  Serial.print(F("Writing EEPROM device number "));
  Serial.println(desiredAddress);

  // Send a command to set the EEPROM device number to
  // the desired address.  This command will only be
  // received by Motorons that are configured to listen
  // to the general call address (which is the default)
  // and will only be obeyed by Motorons that have JMP1
  // shorted to GND.
  mc.writeEepromDeviceNumber(desiredAddress);

  nextAddress = getNextAddress(desiredAddress);
}

void processSerialLine()
{
  switch (lineBuffer[0])
  {
  case 's':
    scanForDevices();
    break;

  case 'w':
    writeAddress();
    break;

  case 'u':
    Serial.println(F("Making new addresses take effect."));
    mc.updateDeviceNumber();
    break;

  default:
    Serial.println(F("Error: Unrecognized command."));
    break;
  }
}

void processSerialByte(uint8_t byteReceived)
{
  if (byteReceived == '\r' || byteReceived == '\n')
  {
    if (lineSize >= sizeof(lineBuffer))
    {
      Serial.println(F("Error: Command too long."));
    }
    else if (lineSize != 0)
    {
      lineBuffer[lineSize] = 0;
      processSerialLine();
    }
    lineSize = 0;
    return;
  }

  if (lineSize < sizeof(lineBuffer))
  {
    lineBuffer[lineSize++] = byteReceived;
  }
}

void loop()
{
  if (Serial.available()) { processSerialByte(Serial.read()); }
}

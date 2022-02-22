// This example sketch provides an interactive utility you can
// use to read the EEPROM device number stored in the
// Motoron Motor Controller and change it.  This is useful if you
// want to set up a system with more than two Motoron controllers
// on the same I2C bus, or if there is an address conflict with
// another device in your system.
//
// This sketch communicates with I2C address 15 by default.
// The simplest way to set up the hardware for using this sketch
// is to disconnect all I2C devices from your Arduino, remove
// the JMP1 jumper from a Motoron Motor Controller
// (forcing it to use I2C address 15), and then connect that
// Motoron to your Arduino.
//
// After uploading this sketch to your Arduino, open the
// Serial Monitor (from the Tools menu of the Arduino IDE).
//
// To read the current EEPROM device number from the device,
// send a line starting with the letter "r".
//
// To change the EEPROM device number, send a line starting with
// "w", followed by the new number, in decimal format.
// The number should be between 0 and 127.
// Whitespace before the number is allowed.
// Example command: w120
//
// This sketch expects each command to be terminated with a
// line-ending character ('\r' or '\n').  The serial monitor
// automatically sends a line-ending character when you click
// the Send button or press Enter.  Do not send quote characters.

#include <Motoron.h>

// To use a different I2C address, change the line below.
// The Motoron uses address 15 when jumper JMP1 is disconnected.
const uint8_t address = 15;

MotoronI2C mc(address);

uint8_t lineSize;
char lineBuffer[40];

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

bool establishCommunication()
{
  mc.enableCrc();
  if (mc.getLastError())
  {
    Serial.print(F("Communication error: "));
    Serial.println(mc.getLastError());
    return false;
  }
  return true;
}

void processSerialLine()
{
  switch (lineBuffer[0])
  {
  case 'r':
    if (!establishCommunication()) { return; }
    Serial.print(F("Read EEPROM device number: "));
    Serial.println(mc.readEepromDeviceNumber());
    break;

  case 'w':
    if (!establishCommunication()) { return; }
    uint8_t desired, actual;
    desired = strtoul(lineBuffer + 1, NULL, 10) & 127;
    mc.writeEepromDeviceNumber(desired);
    actual = mc.readEepromDeviceNumber();
    if (desired == actual)
    {
      Serial.print(F("Wrote EEPROM device number: "));
      Serial.println(actual);
    }
    else
    {
      Serial.print(F("Error: Tried to write EEPROM device number "));
      Serial.print(desired);
      Serial.print(F(" but got "));
      Serial.println(actual);
    }
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

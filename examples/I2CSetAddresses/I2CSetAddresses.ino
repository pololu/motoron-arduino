// This example sketch provides an interactive utility you can use to set the
// I2C addresses for a single Motoron controller or multiple controllers
// connected to the same I2C bus.
//
// After uploading this sketch to your Arduino, open the Serial Monitor
// (from the Tools menu of the Arduino IDE).  The top of the serial monitor
// contains a box where you can type commands and send them to
// this sketch by pressing Ctrl + Enter.
//
// To assign an I2C address to a Motoron, send "a" followed by the address
// (in decimal) while the JMP1 pin of the Motoron you wish to change is
// shorted to GND.  For example, "a17" sets the address of the Motoron to
// decimal 17.
//
// Alternatively, you can send a line that just contains "a" by itself in order
// to have the sketch automatically pick an address for you.
//
// To make the Motoron start using its new address, you can reset it by power
// cycling it, driving its RST line low, or by sending "r".
//
// The "s" command does a simple scan of the bus to see which addresses have
// devices responding.  This can help you make sure you have set up your
// I2C bus correctly.
//
// The "i" command goes further, sending extra commands to detect the Motoron
// devices on the bus and print information about them.  Note that if there are
// devices on your I2C bus that are not Motorons, the commands sent to those
// devices might be misinterpreted and cause undesired behavior.
//
// The "a" and "r" commands use the I2C general call address (0),
// so they might interfere with other devices that use that address, and
// they will not work if you disabled the general call address on a Motoron.
//
// This sketch expects each command to be terminated with a line-ending
// character ('\r' or '\n').  The serial monitor automatically sends a
// line-ending character when you send a message.

#include <Motoron.h>

// Make a MotoronI2C object configured to use the general call
// address (0).
MotoronI2C mc(0);

// The next address this sketch will try to use when you send an
// "a" command to automatically assign an address.
uint8_t nextAddress = 17;

uint8_t lineSize;
char lineBuffer[40];

#ifdef __SAM3X8E__
// Arduino Due uses Wire1
#define WIRE Wire1
#else
#define WIRE Wire
#endif

// This function defines which I2C addresses this sketch is
// allowed to communicate with.
bool allowAddressCommunication(uint8_t address)
{
  // Addresses cannot be larger than 127.
  if (address >= 128) { return false; }

  // If you have devices on your bus and you want to
  // prevent this sketch from talking to them, potentially
  // causing unwanted operations, add their 7-bit addresses here
  // with a line like this:
  // if (address == 0x6B) { return false; }

  return true;
}

// This function defines which I2C addresses this sketch is
// allowed to assign to a device.
bool allowAddressAssignment(uint8_t address)
{
  return allowAddressCommunication(address) && address != 0;
}

void assignAddress()
{
  bool desiredAddressSpecified = false;
  uint8_t desiredAddress = nextAddress;
  if (lineBuffer[1])
  {
    desiredAddress = strtoul(lineBuffer + 1, NULL, 10) & 127;
    desiredAddressSpecified = true;
  }

  while (1)
  {
    if (allowAddressAssignment(desiredAddress))
    {
      if (desiredAddressSpecified || !allowAddressCommunication(desiredAddress))
      {
        break;
      }

      // Make sure there is not already a device on the bus using
      // the desired address.
      WIRE.beginTransmission(desiredAddress);
      uint8_t error = WIRE.endTransmission();
      if (error == 2) { break; }

      Serial.print(F("Found a device at address "));
      Serial.print(desiredAddress);
      Serial.println('.');
    }
    else if (desiredAddressSpecified)
    {
      Serial.print(F("Assignment to address "));
      Serial.print(desiredAddress);
      Serial.println(" not allowed.");
      return;
    }

    // Try the next higher address.
    do { desiredAddress++; }
    while(!allowAddressAssignment(desiredAddress));
  }

  mc.enableCrc();

  // Send a command to set the EEPROM device number to
  // the desired address.  This command will only be
  // received by Motorons that are configured to listen
  // to the general call address (which is the default)
  // and will only be obeyed by Motorons that have JMP1
  // shorted to GND.
  mc.writeEepromDeviceNumber(desiredAddress);

  Serial.print(F("Assigned address "));
  Serial.println(desiredAddress);

  nextAddress = desiredAddress + 1;
}

void scanForDevices()
{
  Serial.println(F("Scanning for I2C devices..."));
  for (uint8_t i = 0; i < 128; i++)
  {
    if (!allowAddressCommunication(i)) { continue; }

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

// Note: This routine writes to many different addresses on your
// I2C bus.  If you have any non-Motoron devices on your bus,
// this could cause unexpected changes to those devices.
// You can modify allowAddressCommunication() to prevent this.
void identifyDevices()
{
  char buffer[80];
  const char * jumperString;

  Serial.println(F("Identifying Motoron controllers..."));
  for (uint8_t i = 1; i < 128; i++)
  {
    if (!allowAddressCommunication(i)) { continue; }

    MotoronI2C test(i);
    test.enableCrc();
    if (test.getLastError()) { continue; }

    uint16_t productId, firmwareVersion;
    test.getFirmwareVersion(&productId, &firmwareVersion);
    if (test.getLastError()) { continue; }

    uint8_t jumperState = test.getJumperState() & 3;
    if (jumperState == 0b10)
    {
      jumperString = "off";
      // JMP1 is high, not connected to GND.
    }
    else if (jumperState == 0b01)
    {
      jumperString = "on";
      // JMP1 is low, connected to GND.
    }
    else if (jumperState == 0b00)
    {
      jumperString = "both";
      // There are multiple Motorons using this address and
      // their JMP1 pins are in different states.
    }
    else
    {
      jumperString = "err";
    }

    sprintf(buffer, "%3d: product=0x%04X version=%x.%02x JMP1=%s",
      i, productId, firmwareVersion >> 8, firmwareVersion & 0xFF,
      jumperString);
    Serial.println(buffer);
  }
  Serial.println(F("Done."));
}

void processSerialLine()
{
  switch (lineBuffer[0])
  {
  case 'a':
    assignAddress();
    break;

  case 'r':
    Serial.println(F("Reset"));
    mc.reset();
    break;

  case 's':
    scanForDevices();
    break;

  case 'i':
    identifyDevices();
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

  if (lineSize < sizeof(lineBuffer) && byteReceived != '"')
  {
    lineBuffer[lineSize++] = byteReceived;
  }
}

void setup()
{
  Serial.begin(9600);
  WIRE.begin();
  mc.setBus(&WIRE);
}

void loop()
{
  if (Serial.available()) { processSerialByte(Serial.read()); }
}

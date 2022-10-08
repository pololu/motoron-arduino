// This example sketch provides an interactive utility you can use to set the
// device number and baud rate for a single Motoron controller or
// multiple controllers.
//
// After uploading this sketch to your Arduino, open the Serial Monitor
// (from the Tools menu of the Arduino IDE).
//
// The top of the serial monitor contains a box where you can type commands for
// this sketch and send them to this sketch by clicking the "Send" button.
//
// In general, for any of the commands described below to work, the Motoron
// must be wired properly to the Arduino so it can receive serial
// commands and send responses.  It must also be operating at the baud rate
// specified by the useBaudRate number below.  (If you are not sure what
// baud rate your Motoron is using, you can short JMP1 to GND and then
// power cycle or reset it to ensure it uses 9600 baud.)
//
// To set up a Motoron, send "a" followed by the desired device number
// (in decimal) and press Enter.  This command writes to the EEPROM of the
// Motoron, settings it device number to the specified value and setting
// its baud rate to the setBaudRate number configured below.
// In order for this command to work, the Motoron's JMP1 pin must be shorted
// to GND.
//
// For example, "a17" sets the device number of the Motoron to decimal 17
// and sets its baud rate to useBaudRate (9600 by default).
//
// To make the Motoron start using its new device number and/or baud rate, you
// can reset it by power cycling it, driving its RST line low, or by
// sending a reset command to it using "r".
//
// The "i" command identifies all Motorons connected to the serial bus and
// prints prints information about them.  If you have multiple Motorons
// connected to the serial bus, each one needs to have a different device number
// or else this command will not work properly.
//
//
// This sketch expects each command to be terminated with a line-ending
// character ('\r' or '\n').  The serial monitor automatically sends a
// line-ending character when you click the Send button or press Enter.

// The baud rate to use when sending serial commands.
const uint32_t useBaudRate = 9600;

// The baud rate we will configure each Motoron to use.
const uint32_t setBaudRate = 9600;

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

uint8_t lineSize;
char lineBuffer[40];

void assignSettings()
{
  uint8_t deviceNumber = strtoul(lineBuffer + 1, NULL, 10) & 127;

  mc.enableCrc();

  // This command will only be obeyed by the Motoron if its JMP1 pin
  // is shorted to GND.
  mc.writeEepromDeviceNumber(deviceNumber);
  if (setBaudRate) { mc.writeEepromBaudRate(setBaudRate); }

  Serial.print(F("Assigned device number "));
  Serial.print(deviceNumber);
  if (setBaudRate)
  {
    Serial.print(F(" and baud rate "));
    Serial.print(setBaudRate);
  }
  Serial.println();
}

// Note: This routine writes to many different addresses on your
// I2C bus.  If you have any non-Motoron devices on your bus,
// this could cause unexpected changes to those devices.
// You can modify allowAddressCommunication() to prevent this.
void identifyDevices()
{
  char buffer[80];
  const char * jumperString;

  Serial.print(F("Identifying Motoron controllers ("));
  Serial.print(useBaudRate);
  Serial.println(F(" baud)..."));
  for (uint8_t i = 0; i < 128; i++)
  {
    // Flush any incoming data that was received earlier.
    while (mcSerial.available()) { mcSerial.read(); }

    mc.setDeviceNumber(i);
    mc.enableCrc();

    uint16_t productId, firmwareVersion;
    mc.getFirmwareVersion(&productId, &firmwareVersion);
    if (mc.getLastError()) { continue; }

    uint8_t jumperState = mc.getJumperState() & 3;
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

  mc.setDeviceNumber(255);
}

void processSerialLine()
{
  switch (lineBuffer[0])
  {
  case 'a':
    assignSettings();
    break;

  case 'r':
    Serial.println(F("Reset"));
    mc.reset();
    break;

  case 's':
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
  mcSerial.begin(useBaudRate);
  mcSerial.setTimeout(20);
  mc.setPort(&mcSerial);
}

void loop()
{
  if (Serial.available()) { processSerialByte(Serial.read()); }
}

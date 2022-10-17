// This example sketch provides an interactive utility you can use to set the
// device number and baud rate for a single serial Motoron controller or
// multiple serial Motoron controllers.
//
// For this sketch to work the Motoron must be wired properly to the Arduino so
// it can receive serial commands.  It must also be operating at a known baud
// rate, and the baud rate must be one that your Arduino is capable of
// generating accurately.  (If you are not sure what baud rate your
// Motoron is using or whether your Arduino can generate it accurately, you can
// short JMP1 to GND and then power cycle or reset the Motoron to ensure it
// uses 9600 baud.)
//
// After uploading this sketch to your Arduino, open the Serial Monitor
// (from the Tools menu of the Arduino IDE).  The top of the serial monitor
// contains a box where you can type commands and send them to
// this sketch by pressing Ctrl + Enter.
//
// This sketch uses 115200 baud to talk to the Motorons by default.  If your
// Motoron is using a different baud rate, send "u" followed by the baud rate
// in decimal (for example "u9600") to reconfigure the sketch.
//
// To set the device number in the Motoron's non-volatile EEPROM memory,
// send "a" followed by the desired device number in decimal (for example
// "a17").  In order for this command to work, the Motoron's JMP1 pin must be
// shorted to GND.
//
// Alternatively, you can send a line that just contains "a" by itself in order
// to have the sketch automatically pick a device number for you.
//
// To set the baud rate in the Motoron's non-volatile EEPROM memory, send
// "b" followed by the desired baud rate (in decimal).
// In order for this command to work, the Motoron's JMP1 pin must be shorted
// to GND.
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
// This sketch expects each command to be terminated with a line-ending
// character ('\r' or '\n').  The serial monitor automatically sends a
// line-ending character when you send a message.

#include <Motoron.h>

uint32_t useBaudRate = 115200;

uint8_t nextDeviceNumber = 17;

uint32_t nextBaudRate = 115200;

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

void assignDeviceNumber()
{
  uint8_t deviceNumber = nextDeviceNumber;
  if (lineBuffer[1])
  {
    deviceNumber = strtoul(lineBuffer + 1, NULL, 10) & 127;
  }

  mc.enableCrc();

  // This command will only be obeyed by the Motoron if its JMP1 pin
  // is shorted to GND.
  mc.writeEepromDeviceNumber(deviceNumber);

  Serial.print(F("Assigned device number "));
  Serial.print(deviceNumber);
  Serial.println('.');
  nextDeviceNumber = (deviceNumber + 1) & 127;
}

void assignBaudRate()
{
  uint32_t baudRate = nextBaudRate;
  if (lineBuffer[1])
  {
    baudRate = strtoul(lineBuffer + 1, NULL, 10);
  }
  if (baudRate < MOTORON_MIN_BAUD_RATE || baudRate > MOTORON_MAX_BAUD_RATE)
  {
    Serial.println(F("Invalid baud rate."));
    return;
  }

  mc.enableCrc();

  // This command will only be obeyed by the Motoron if its JMP1 pin
  // is shorted to GND.
  mc.writeEepromBaudRate(baudRate);

  Serial.print(F("Assigned "));
  Serial.print(baudRate);
  Serial.println(F(" baud."));

  nextBaudRate = baudRate;
}

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

void useNewBaudRate()
{
  uint32_t baudRate = strtoul(lineBuffer + 1, NULL, 10);
  if (baudRate < MOTORON_MIN_BAUD_RATE || baudRate > MOTORON_MAX_BAUD_RATE)
  {
    Serial.println(F("Invalid baud rate."));
    return;
  }

  useBaudRate = baudRate;
  mcSerial.begin(useBaudRate);

  Serial.print(F("Using "));
  Serial.print(useBaudRate);
  Serial.println(F(" baud."));
}

void processSerialLine()
{
  switch (lineBuffer[0])
  {
  case 'a':
    assignDeviceNumber();
    break;

  case 'b':
    assignBaudRate();
    break;

  case 'r':
    Serial.println(F("Reset"));
    mc.reset();
    break;

  case 'i':
    identifyDevices();
    break;

  case 'u':
    useNewBaudRate();
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

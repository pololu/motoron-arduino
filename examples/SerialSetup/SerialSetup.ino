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
// Command          | Summary
// -----------------|----------------------------------------------------------
// a [NUM] [ALTNUM] | Assign all serial parameters (JMP1 must be low).
// r                | Reset devices.
// i                | Identify devices.
// b BAUD           | Use a different baud rate to communicate.
// o [OPTS]         | Use different serial options to communicate.
// n                | Use 115200 baud, 8-bit responses, 7-bit device number.
// j                | Use 9600 baud, 8-bit responses, 7-bit device number.
// k                | Use the options & baud rate we're assigning to devices.
//
// This sketch expects each command to be terminated with a line-ending
// character ('\r' or '\n').
//
// For more information, see the "Setting up serial parameters with an Arduino"
// section of the Motoron user's guide or read the comments below.

#include <Motoron.h>

// The serial parameters below are assigned to a device when you use the "a"
// command.  You can modify these lines to assign different parameters.
const uint32_t assignBaudRate = 115200;
const bool assign7BitResponses = false;
const bool assign14BitDeviceNumber = false;
const bool assignErrIsDE = false;
const uint8_t assignResponseDelay = 0;

static_assert(assignBaudRate >= MOTORON_MIN_BAUD_RATE &&
  assignBaudRate <= MOTORON_MAX_BAUD_RATE, "Invalid baud rate.");

const uint8_t assignSerialOptions =
    (assign7BitResponses << MOTORON_SERIAL_OPTION_7BIT_RESPONSES) |
    (assign14BitDeviceNumber << MOTORON_SERIAL_OPTION_14BIT_DEVICE_NUMBER) |
    (assignErrIsDE << MOTORON_SERIAL_OPTION_ERR_IS_DE);

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

uint32_t useBaudRate = 115200;
uint16_t nextDeviceNumber = 17;

uint8_t lineSize;
char lineBuffer[40];

// Command syntax: a [NUM] [ALTNUM]
//
// This command sends a series of "Write EEPROM" commands using the
// compact protocol to set all the serial parameters on a Motoron.
// For this command to work, the Motoron must be using the same baud rate as
// the Arduino, and the Motoron's JMP1 line must be low.
//
// NUM should be the desired device number for the Motoron, written in decimal.
// If NUM is omitted or equal to "-1", the sketch will automatically pick a
// device number.
//
// ALTNUM should be the desired *alternative* device number for the Motoron.
// If ALTNUM is omitted or equal to "-1", the feature is disabled.
//
// The other serial parameters (e.g. assignBaudRate) are set according to the
// constants above.
//
// This command does not have an immediate affect, but you can use the "r"
// command to reset the Motoron and make the new settings take effect.
//
// Examples:
//   a
//   a 17
//   a -1 0
void assignAllSerialParameters()
{
  uint16_t maxDeviceNumber = assign14BitDeviceNumber ? 0x3FFF : 0x7F;

  uint16_t deviceNumber = nextDeviceNumber;
  uint16_t altDeviceNumber = 0xFFFF;

  // TODO: add a call to sscanf here but see how much program space it takes
  // TODO: parse the user input to set deviceNumber and altDeviceNumber

  mc.enableCrc();
  mc.writeEepromDeviceNumber(deviceNumber);
  if (altDeviceNumber <= maxDeviceNumber)
  {
    mc.writeEepromAlternativeDeviceNumber(altDeviceNumber);
  }
  else
  {
    mc.writeEepromDisableAlternativeDeviceNumber();
  }
  mc.writeEepromBaudRate(assignBaudRate);
  mc.writeEepromSerialOptions(assignSerialOptions);
  mc.writeEepromResponseDelay(assignResponseDelay);

  Serial.print(F("Assigned "));
  if (assign14BitDeviceNumber)
  {
    Serial.print(F(" 14-bit"));
  }
  Serial.print(F("device number "));
  Serial.print(deviceNumber);
  if (altDeviceNumber <= maxDeviceNumber)
  {
    Serial.print(F(" and "));
    Serial.print(altDeviceNumber);
  }
  Serial.print(F(", "));
  Serial.print(assignBaudRate);
  Serial.print(F(" baud"));
  if (assign7BitResponses)
  {
    Serial.print(F(", 7-bit responses"));
  }
  else
  {
    Serial.print(F(", 8-bit responses"));
  }
  if (assignErrIsDE)
  {
    Serial.print(F(", ERR is DE"));
  }
  else
  {
    Serial.print(F(", ERR is normal"));
  }
  Serial.println('.');

  nextDeviceNumber = (deviceNumber + 1) & maxDeviceNumber;
}

void printCommunicationSettings()
{
  Serial.print(useBaudRate);
  Serial.print(F(" baud"));
  if (mc.getSerialOptionsLocally() & (1 << MOTORON_SERIAL_OPTION_14BIT_DEVICE_NUMBER))
  {
    Serial.print(F(", 14-bit device number"));
  }
  else
  {
    Serial.print(F(", 7-bit device number"));
  }
  if (mc.getSerialOptionsLocally() & (1 << MOTORON_SERIAL_OPTION_7BIT_RESPONSES))
  {
    Serial.print(F(", 7-bit responses"));
  }
  else
  {
    Serial.print(F(", 8-bit responses"));
  }
}

void printCommunicationSettingsLine()
{
  Serial.print(F("Using "));
  printCommunicationSettings();
  Serial.println(F("."));
}

void printDeviceInfoIfPossible()
{
  uint16_t productId, firmwareVersion;
  mc.getFirmwareVersion(&productId, &firmwareVersion);
  if (mc.getLastError()) { return; }

  uint8_t jumperState = mc.getJumperState() & 3;
  const char * jumperString;
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

  // We fetch EEPROM with two requests because the maximum payload size in
  // 7-bit resopnse mode is 7 bytes.
  uint8_t eeprom[8] = { 0 };
  mc.readEeprom(1, 4, eeprom + 0);
  mc.readEeprom(5, 4, eeprom + 4);

  char buffer[80];
  sprintf(buffer, "%3d: product=0x%04X version=%x.%02x JMP1=%s",
    mc.getDeviceNumber(), productId,
    firmwareVersion >> 8, firmwareVersion & 0xFF, jumperString);
  Serial.print(buffer);

  Serial.print(F(" EEPROM="));
  for (uint8_t i = 0; i < sizeof(eeprom); i++)
  {
    if (eeprom[i] < 16) { Serial.print('0'); }
    Serial.print(eeprom[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// Command syntax: i
//
// This command tries to communicate with every possible device number using
// the current communication settings and the Pololu protocol.  If it finds a
// Motoron it prints out some information about it.
//
// Note: When using 14-bit device numbers, this command will take a long time.  (TODO: how long?)
void identifyDevices()
{
  uint16_t maxDeviceNumber =
    mc.getSerialOptionsLocally() & (1 << MOTORON_SERIAL_OPTION_14BIT_DEVICE_NUMBER)
    ? 0x3FFF : 0x7F;

  Serial.print(F("Identifying Motoron controllers ("));
  printCommunicationSettings();
  Serial.println(F(")..."));
  for (uint16_t i = 0; i <= maxDeviceNumber; i++)
  {
    // Flush any incoming data that was received earlier.
    while (mcSerial.available()) { mcSerial.read(); }

    mc.setDeviceNumber(i);
    mc.enableCrc();
    printDeviceInfoIfPossible();
  }
  Serial.println(F("Done."));

  mc.setDeviceNumber(0xFFFF);
}

void setBaudRate(uint32_t baudRate)
{
  useBaudRate = baudRate;
  mcSerial.begin(useBaudRate);
}

// Command syntax: b BAUD
//
// This command configures the Arduino to use a different baud rate when
// communicating with the Motoron.
//
// Example: b 38600
void commandSetBaudRate()
{
  uint32_t baudRate = strtoul(lineBuffer + 1, NULL, 10);
  if (baudRate < MOTORON_MIN_BAUD_RATE || baudRate > MOTORON_MAX_BAUD_RATE)
  {
    Serial.println(F("Invalid baud rate."));
    return;
  }
  setBaudRate(baudRate);
  printCommunicationSettingsLine();
}

// Command syntax: o [OPTS]
//
// This command configures the Arduino to use different serial options when
// communicating with the Motoron.
//
// If OPTS is omitted, this command cycles through the four different possible
// sets of serial options.
//
// Otherwise, OPTS should be a decimal number between 0 and 3:
//   0 = 8-bit responses, 7-bit device numbers
//   1 = 7-bit responses, 7-bit device numbers
//   2 = 8-bit responses, 14-bit device numbers
//   3 = 7-bit responses, 14-bit device numbers
void commandSetSerialOptions()
{
  uint8_t options;
  if (lineBuffer[1])
  {
    options = strtoul(lineBuffer + 1, NULL, 10);
  }
  else
  {
    options = (mc.getSerialOptionsLocally() + 1) & 3;
  }
  mc.setSerialOptions(options);
  printCommunicationSettingsLine();
}

void processSerialLine()
{
  switch (lineBuffer[0])
  {
  case 'a':
    assignAllSerialParameters();
    break;

  case 'r':
    Serial.println(F("Reset"));
    mc.reset();
    break;

  case 'i':
    identifyDevices();
    break;

  case 'b':
    commandSetBaudRate();
    break;

  case 'o':
    commandSetSerialOptions();
    break;

  case 'n':
    setBaudRate(115200);
    mc.setSerialOptions(0);
    printCommunicationSettingsLine();
    break;

  case 'j':
    setBaudRate(9600);
    mc.setSerialOptions(0);
    printCommunicationSettingsLine();
    break;

  case 'k':
    setBaudRate(assignBaudRate);
    mc.setSerialOptions(assignSerialOptions);
    printCommunicationSettingsLine();
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
  mcSerial.setTimeout(2);
  mc.setPort(&mcSerial);
}

void loop()
{
  if (Serial.available()) { processSerialByte(Serial.read()); }
}

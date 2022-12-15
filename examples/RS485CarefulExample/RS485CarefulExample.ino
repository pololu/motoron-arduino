#include <Motoron.h>

#define DE_PIN 9
#define RE_PIN 10

class SerialWithDE : public Stream
{
public:
  SerialWithDE(HardwareSerial * serial, uint8_t de_pin = 0xFF, uint8_t re_pin = 0xFF)
  : hws(serial), de_pin(de_pin), re_pin(re_pin)
  {
  }

  void begin(unsigned long baud) { hws->begin(baud); }

  void txMode()
  {
    if (de_pin != 0xFF) { digitalWrite(de_pin, HIGH); }
    if (re_pin != 0xFF) { digitalWrite(re_pin, HIGH); }
  }

  void rxMode()
  {
    if (de_pin != 0xFF) { digitalWrite(de_pin, LOW); }
    if (re_pin != 0xFF) { digitalWrite(re_pin, LOW); }
  }

  int available() override
  {
    rxMode();
    return hws->available();
  }

  int peek() override
  {
    rxMode();
    return hws->peek();
  }

  int read() override
  {
    rxMode();
    return hws->read();
  }

  int availableForWrite() override
  {
    return hws->availableForWrite();
  }

  void flush() override
  {
    hws->flush();
  }

  size_t write(uint8_t b) override
  {
    txMode();
    return hws->write(b);
  }

  HardwareSerial * hws;
  uint8_t de_pin, re_pin;
};

SerialWithDE mcSerial(&SERIAL_PORT_HARDWARE_OPEN, 9, 10);

MotoronSerial mc;

void setup() {
  mcSerial.begin(115200);
  mcSerial.setTimeout(20);

  mc.setPort(&mcSerial);
  mc.setDeviceNumber(17);
  mc.expect7BitResponses();

  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  pinMode(RE_PIN, OUTPUT);
  digitalWrite(RE_PIN, LOW);
}

void loop() {
  while (mcSerial.available()) { mcSerial.read(); }
  mc.reinitialize();
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
  mc.clearResetFlag();
  mc.setSpeedNow(1, 100);
  delay(250);
}

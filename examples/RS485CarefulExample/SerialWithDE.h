#pragma once

// This class wraps HardwareSerial object and allows you to drive one or two
// pins high whenever the serial object is transmitting in order to control
// an RS-485 driver and receiver.
//
// Ideally this object would switch automatically switch from transmit mode to
// receive mode when the transmission is done, but it does not because it would
// be hard to get that working in a portable way.  You must call rxMode(),
// available(), peek(), or read() to go back into receive mode.
class SerialWithDE : public Stream
{
public:
  SerialWithDE(HardwareSerial * serial, uint8_t de_pin = 0xFF, uint8_t re_pin = 0xFF)
  : hws(serial), de_pin(de_pin), re_pin(re_pin)
  {
  }

  void begin(unsigned long baud)
  {
    if (de_pin != 0xFF)
    {
      pinMode(de_pin, OUTPUT);
      digitalWrite(de_pin, LOW);
    }
    if (re_pin != 0xFF)
    {
      pinMode(re_pin, OUTPUT);
      digitalWrite(re_pin, LOW);
    }
    hws->begin(baud);
  }

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
    // TODO: rxMode() here?
  }

  size_t write(uint8_t b) override
  {
    txMode();
    return hws->write(b);
  }

  HardwareSerial * hws;
  uint8_t de_pin, re_pin;
};

#pragma once

// This class wraps HardwareSerial object and drives one or two configurable
// pins high whenever the serial object is transmitting.  These pins
// essentially output the same signal, and you can connect that signal to the
// the DE (Driver Enable) and RE (Receiver Enable) pins on an RS-485
// transceiver chip.
//
// This object switchs to transmit mode when you call txMode() or write()
// (note that write() is called by every function that writes or prints any
// data to the serial port.)
//
// Unforunately, it does not switch to receive mode automatically when the
// transmission is done.  You must call rxMode() or flush() to do that.
// This class works well with classes like the MotoronSerial which
// calls flush() before reading any responses.
//
// A future, smarter version of this class might keep track of whether it is
// in TX mode or RX mode.  Then in the available(), read(), and peek()
// functions, it should call flush() *if* it is currently in TX mode.
// However, this implementation is good enough for MotoronSerial, which always
// calls flush().
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
    return hws->available();
  }

  int peek() override
  {
    return hws->peek();
  }

  int read() override
  {
    return hws->read();
  }

  int availableForWrite() override
  {
    return hws->availableForWrite();
  }

  void flush() override
  {
    hws->flush();
    rxMode();
  }

  size_t write(uint8_t b) override
  {
    txMode();
    return hws->write(b);
  }

  HardwareSerial * hws;
  uint8_t de_pin, re_pin;
};

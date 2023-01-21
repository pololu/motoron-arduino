# Motoron Motor Controller library for Arduino

[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that helps interface with
[Motoron motor controllers][motoron] using I&sup2;C or UART serial.

It supports the following Motoron controllers:

- [Motoron M1T256 Single I&sup2;C Motor Controller][M1T256]
- [Motoron M1U256 Single Serial Motor Controller][M1U256]
- [Motoron M2T256 Dual I&sup2;C Motor Controller][M2T256]
- [Motoron M2U256 Dual Serial Motor Controller][M2U256]
- [Motoron M3S256 Triple Motor Controller Shield for Arduino][M3S256]
- [Motoron M3H256 Triple Motor Controller for Raspberry Pi][M3H256]
- [Motoron M2S Dual High-Power Motor Controllers for Arduino][M2S] (M2S18v20, M2S18v18, M2S24v16, M2S24v14)
- [Motoron M2H Dual High-Power Motor Controllers for Raspberry Pi][M2H] (M2H18v20, M2H18v18, M2H24v16, M2H24v14)

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.8.x or later;
we have not tested it with earlier versions.  This library should support any
Arduino-compatible board, including the [Pololu A-Star controllers][a-star].

## Getting started

### Hardware

The Motoron motor controllers can be purchased from Pololu's website.
Before continuing, careful reading of the [Motoron user's guide][guide]
is recommended.

### I&sup2;C connections

To control a Motoron with its I&sup2;C interface (SCL and SDA),
you will need to power the Motoron's logic and connect the Motoron
to your board's I&sup2;C pins.
Plugging the Motoron shield into a standard Arduino-compatible board achieves
this.

If you are not plugging the Motoron in as a shield, you will need to connect
the GND pins of both boards, connect the SDA pins of both boards, connect
the SCL pins of both boards.
You should also supply power to the Motoron's logic voltage pin (which is named
VDD, IOREF, or 3V3) by connecting it to the logic voltage supply of your
controller board, which should be between 3.0&nbsp;V and 5.5&nbsp;V.

### UART serial connections

To control a Motoron with a UART serial interface (RX and TX), you need to
at least connect your board's TX pin (as defined in the table below) to the
Motoron's RX pin, and connect your board's ground pin to one of the Motoron's
GND pins.  If you want to read information from the Motoron, you must also
connect your board's RX pin to the Motoron's TX pin.
You should also supply power to the Motoron's logic voltage pin (which is named
VDD, IOREF, or 3V3) by connecting it to the logic voltage supply of your
controller board, which should be between 3.0&nbsp;V and 5.5&nbsp;V.

The example sketches for this library use a hardware serial port on your Arduino
if one is available: if your Arduino environment defines
`SERIAL_PORT_HARDWARE_OPEN`, the examples will use that port.  The pins for this
serial port are different depending on which Arduino you are using.

| Microcontroller Board | Hardware serial? | MCU RX pin | MCU TX pin |
|-----------------------|------------------|------------|------------|
| A-Star 32U4           |        Yes       |      0     |      1     |
| A-Star 328PB          |        Yes       |     12     |     11     |
| Arduino Leonardo      |        Yes       |      0     |      1     |
| Arduino Micro         |        Yes       |      0     |      1     |
| Arduino Mega 2560     |        Yes       |     19     |     18     |
| Arduino Due           |        Yes       |     19**   |     18     |
| Arduino Uno           |        No        |     10     |     11     |
| Arduino Yun           |        No        |     10     |     11     |


### Software

You can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then
   "Manage Libraries...".
2. Search for "Motoron Motor Controller".
3. Click the Motoron entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub][releases]
   and decompress it.
2. Rename the folder "motoron-arduino-xxxx" to "Motoron".
3. Drag the "Motoron" folder into the "libraries" directory inside your
   Arduino sketchbook directory. You can view your sketchbook location by
   opening the "File" menu and selecting "Preferences" in the Arduino IDE. If
   there is not already a "libraries" folder in that location, you should make
   the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Several example sketches are available that show how to use the library. You can
access them from the Arduino IDE by opening the "File" menu, selecting
"Examples", and then selecting "Motoron". If you cannot find these
examples, the library was probably installed incorrectly and you should retry
the installation instructions above.

## Classes

The main classes provided by this library are MotoronI2C and MotoronSerial.
Each of these is a subclass of MotoronBase.

## Documentation

For complete documentation of this library, see
[the motoron-arduino documentation][doc].
If you are already on that page, then click the links in the "Classes" section above.

## Command timeout

By default, the Motoron will turn off its motors if it has not received a valid
command in the last 1.5 seconds.  You can change the amount of time it
takes for the Motoron to time out using MotoronBase::setCommandTimeoutMilliseconds()
or you can disable the feature using MotoronBase::disableCommandTimeout().

## Version history

* 1.3.0 (2023-01-20): Added support for the [M1T256] and [M1U256] motorons.
* 1.2.0 (2022-12-16): Added support for the [M2T256] and [M2U256] motorons.
* 1.1.0 (2022-07-22): Added support for the [M2S] and [M2H] Motorons.
* 1.0.0 (2022-03-25): Original release.

[motoron]: https://pololu.com/motoron
[M1T256]: https://www.pololu.com/product/5061
[M1U256]: https://www.pololu.com/product/5063
[M2T256]: https://www.pololu.com/product/5065
[M2U256]: https://www.pololu.com/product/5067
[M3S256]: https://www.pololu.com/category/290
[M3H256]: https://www.pololu.com/category/292
[M2S]: https://www.pololu.com/category/291
[M2H]: https://www.pololu.com/category/293
[a-star]: https://www.pololu.com/a-star
[releases]: https://github.com/pololu/motoron-arduino/releases
[doc]: https://pololu.github.io/motoron-arduino/
[guide]: https://www.pololu.com/docs/0J84
[ide]: https://www.arduino.cc/en/Main/Software

# Motoron Motor Controller library for Arduino

[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that helps interface with a
Motoron motor controller using I&sup2;C.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.8.x or later;
we have not tested it with earlier versions.  This library should support any
Arduino-compatible board, including the [Pololu A-Star controllers][a-star].

## Getting started

### Hardware

The Motoron motor controllers can be purchased from Pololu's website.
Before continuing, careful reading of the [Motoron user's guide][guide]
is recommended.

### Connections

In order to use this library, you will need to power the Motoron's
logic and connect the Motoron to your board's I&sup2;C pins.  Plugging the
Motoron shield into a standard Arduino-compatible board achieves this.

If you are not plugging the Motoron in as a shield, you will need to connect
the GND pins of both boards, connect the SDA pins of both boards, connect
the SCL pins of both boards, and connect the IOREF pins of both boards.

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

The main class provided by this library is MotoronI2C.

## Documentation

For complete documentation of this library, see [the motoron-arduino documentation][doc].  If you are already on that page, then click the links in the "Classes" section above.

[a-star]: https://www.pololu.com/a-star
[releases]: https://github.com/pololu/motoron-arduino/releases
[doc]: https://pololu.github.io/motoron-arduino/
[guide]: https://www.pololu.com/docs/0J84
[ide]: https://www.arduino.cc/en/Main/Software

## Command timeout

By default, the Motoron will turn off its motors if it has not received a valid command in the last 1.5 seconds.  You can change the amount of time it
takes for the Motoron to time out using MotoronI2C::setCommandTimeoutMilliseconds()
or you can disable the feature using
MotoronI2C::disableCommandTimeout().

## Version history

* 1.0.0 (2022-03-25): Original release.

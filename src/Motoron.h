// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

/// \file Motoron.h
///
/// This is the main header file for the Motoron Motor Controller library
/// for Arduino.
///
/// For more information about the library, see the main repository at:
/// https://github.com/pololu/motoron-arduino

/// \file motoron_protocol.h
///
/// This file defines the arbitrary constants needed to communicate with a
/// Motoron.

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "motoron_protocol.h"

/// \cond

extern const PROGMEM uint8_t motoronCrcTable[256];

/// \endcond

/// Specifies what type of Motoron is being used, for the purposes of current
/// limit and current sense calculations.
enum class MotoronCurrentSenseType {
  Motoron18v18 = 0b0001,
  Motoron24v14 = 0b0101,
  Motoron18v20 = 0b1010,
  Motoron24v16 = 0b1101,
};

struct MotoronCurrentSenseReading
{
  uint16_t raw;
  int16_t speed;
  uint16_t processed;
};

/// This is a base class used to represent a connection to a Tic.  This class
/// provides high-level functions for sending commands to the Tic and reading
/// data from it.
///
/// \sa MotoronSerial, MotoronI2C
class MotoronBase
{
public:
  MotoronBase()
  {
    lastError = 0;
    protocolOptions = defaultProtocolOptions;
  }

  /// Returns 0 if the last communication with the device was successful, or
  /// a non-zero error code if there was an error.
  uint8_t getLastError()
  {
    return lastError;
  }

  /// Sends the "Get firmware version" command to get the device's firmware
  /// product ID and firmware version numbers.
  ///
  /// For more information, see the "Get firmware version"
  /// command in the Motoron user's guide.
  ///
  /// \param productId An optional pointer used to return the product ID.
  ///   Can be NULL.
  /// \param firmwareVersion An optional pointer used to return the firmware
  ///   version.  Can be NULL.
  void getFirmwareVersion(uint16_t * productId, uint16_t * firmwareVersion)
  {
    uint8_t cmd = MOTORON_CMD_GET_FIRMWARE_VERSION;
    uint8_t response[4];
    sendCommandAndReadResponse(1, &cmd, sizeof(response), response);
    if (productId != nullptr) { *productId = response[0] | (response[1] << 8); }
    if (firmwareVersion != nullptr) { *firmwareVersion = response[2] | (response[3] << 8); }
  }

  /// Sends the "Set protocol options" command to the device to specify options
  /// related to how the device processes commands and sends responses.
  /// The options are also saved in this object and are used later
  /// when sending other commands or reading responses.
  ///
  /// When CRC for commands is enabled, this library generates the CRC
  /// byte and appends it to the end of each command it sends.  The Motoron
  /// checks it to help ensure the command was received correctly.
  ///
  /// When CRC for responses is enabled, this library reads the CRC byte sent
  /// by the Motoron in its repsonses and makes sure it is correct.  If the
  /// response CRC byte is incorrect, getLastError() will return a non-zero
  /// error code after the command has been run.
  ///
  /// When the I2C general call address is enabled, the Motoron receives
  /// commands sent to address 0 in addition to its usual I2C address.
  /// The general call address is write-only; reading bytes from it is not
  /// supported.
  ///
  /// By default (in this libary and the Motoron itself), CRC for commands and
  /// responses is enabled, and the I2C general call address is enabled.
  ///
  /// This method always sends its command with a CRC byte, so it will work
  /// even if CRC was previously disabled but has been re-enabled on the device
  /// (e.g. due to a reset).
  ///
  /// The \p options argument should be 0 or a combination of the following
  /// expressions made using the bitwise or operator (|):
  /// - (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS)
  /// - (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES)
  /// - (1 << MOTORON_PROTOCOL_OPTION_I2C_GENERAL_CALL)
  ///
  /// For more information, see the "Set protocol optons"
  /// command in the Motoron user's guide.
  ///
  /// \sa enableCrc(), disableCrc(),
  ///   enableCrcForCommands(), disableCrcForCommands(),
  ///   enableCrcForResponses(), disableCrcForResponses(),
  ///   enableI2cGeneralCall(), disableI2cGeneralCall()
  void setProtocolOptions(uint8_t options)
  {
    this->protocolOptions = options;
    uint8_t cmd[] = {
      MOTORON_CMD_SET_PROTOCOL_OPTIONS,
      (uint8_t)(options & 0x7F),
      (uint8_t)(~options & 0x7F),
    };
    sendCommandCore(sizeof(cmd), cmd, true);
    if (getLastError()) { return; }
  }

  /// Sets the protocol options for this object, without sending a command to
  /// the Motoron.
  ///
  /// If the options you specify here do not match the actual configuration of
  /// the Motoron, future communication could fail.
  ///
  /// Most users should use setProtocolOptions() instead of this.
  void setProtocolOptionsLocally(uint8_t options)
  {
    this->protocolOptions = options;
  }

  /// Enables CRC for commands and responses.  See setProtocolOptions().
  void enableCrc()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS)
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  /// Disables CRC for commands and responses.  See setProtocolOptions().
  void disableCrc()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS)
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  /// Enables CRC for commands.  See setProtocolOptions().
  void enableCrcForCommands()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS));
  }

  /// Disables CRC for commands.  See setProtocolOptions().
  void disableCrcForCommands()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS));
  }

  /// Enables CRC for responses.  See setProtocolOptions().
  void enableCrcForResponses()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  /// Disables CRC for responses.  See setProtocolOptions().
  void disableCrcForResponses()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  /// Enables the I2C general call address.  See setProtocolOptions().
  void enableI2cGeneralCall()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_I2C_GENERAL_CALL));
  }

  /// Disables the I2C general call address.  See setProtocolOptions().
  void disableI2cGeneralCall()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_I2C_GENERAL_CALL));
  }

  /// Reads the specified bytes from the Motoron's EEPROM memory.
  ///
  /// For more information, see the "Read EEPROM" command in the
  /// Motoron user's guide.
  void readEeprom(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_READ_EEPROM,
      (uint8_t)(offset & 0x7F),
      (uint8_t)(length & 0x7F),
    };
    sendCommandAndReadResponse(sizeof(cmd), cmd, length, buffer);
  }

  /// Reads the EEPROM device number from the device.
  /// This is the I2C address that the device uses if it detects that JMP1
  /// is shorted to GND when it starts up.  It is stored in non-volatile
  /// EEPROM memory.
  uint8_t readEepromDeviceNumber()
  {
    uint8_t number;
    readEeprom(MOTORON_SETTING_DEVICE_NUMBER, 1, &number);
    return number;
  }

  /// Writes a value to one byte in the Motoron's EEPROM memory.
  ///
  /// **Warning: Be careful not to write to the EEPROM in a fast loop. The
  /// EEPROM memory of the Motoron’s microcontroller is only rated for
  /// 100,000 erase/write cycles.**
  ///
  /// For more information, see the "Write EEPROM" command in the
  /// Motoron user's guide.
  void writeEeprom(uint8_t offset, uint8_t value)
  {
    uint8_t cmd[7];
    cmd[0] = MOTORON_CMD_WRITE_EEPROM;
    cmd[1] = offset & 0x7F;
    cmd[2] = value & 0x7F;
    cmd[3] = value >> 7 & 1;
    cmd[4] = cmd[1] ^ 0x7F;
    cmd[5] = cmd[2] ^ 0x7F;
    cmd[6] = cmd[3] ^ 0x7F;
    sendCommand(sizeof(cmd), cmd);
    flushTransmission();
    delay(6);
  }

  /// Writes to the EEPROM device number, changing it to the specified value.
  ///
  /// **Warning: Be careful not to write to the EEPROM in a fast loop. The
  /// EEPROM memory of the Motoron’s microcontroller is only rated for
  /// 100,000 erase/write cycles.**
  ///
  /// For more information, see the "Write EEPROM" command in the
  /// Motoron user's guide.  Also, see the WriteEEPROM example that comes with
  /// this library for an example of how to use this method.
  void writeEepromDeviceNumber(uint8_t number)
  {
    writeEeprom(MOTORON_SETTING_DEVICE_NUMBER, number);
  }

  /// Sends a "Reinitialize" command to the Motoron, which resets most of the
  /// Motoron's variables to their default state.
  ///
  /// For more information, see the "Reinitialize" command in the Motoron
  /// user's guide.
  ///
  /// \sa reset()
  void reinitialize()
  {
    // Always send the reset command with a CRC byte to make it more reliable.
    uint8_t cmd = MOTORON_CMD_REINITIALIZE;
    sendCommandCore(1, &cmd, true);
    protocolOptions = defaultProtocolOptions;
  }

  /// Sends a "Reset" command to the Motoron, which does a full hardware reset.
  ///
  /// This command is equivalent to briefly driving the Motoron's RST pin low.
  /// The Motoron's RST is briefly driven low by the Motoron itself as a
  /// result of this command.
  ///
  /// After running this command, we recommend waiting for at least 5
  /// milliseconds before you try to communicate with the Motoron.
  ///
  /// \sa reinitialize()
  void reset()
  {
    uint8_t cmd = MOTORON_CMD_RESET;
    sendCommandCore(1, &cmd, true);
    flushTransmission();
    protocolOptions = defaultProtocolOptions;
  }

  /// Reads information from the Motoron using a "Get variables" command.
  ///
  /// This library has helper methods to read every variable, but this method
  /// is useful if you want to get the raw bytes, or if you want to read
  /// multiple consecutive variables at the same time for efficiency.
  ///
  /// \param motor 0 to read general variables, or a motor number to read
  ///   motor-specific variables.
  /// \param offset The location of the first byte to read.
  /// \param length How many bytes to read.
  /// \param buffer A pointer to an array to store the bytes read
  ///   from the controller.
  void getVariables(uint8_t motor, uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_GET_VARIABLES,
      (uint8_t)(motor & 0x7F),
      (uint8_t)(offset & 0x7F),
      (uint8_t)(length & 0x7F),
    };
    sendCommandAndReadResponse(sizeof(cmd), cmd, length, buffer);
  }

  /// Reads one byte from the Motoron using a "Get variables" command.
  ///
  /// \param motor 0 to read a general variable, or a motor number to read
  ///   a motor-specific variable.
  /// \param offset The location of the byte to read.
  uint8_t getVar8(uint8_t motor, uint8_t offset)
  {
    uint8_t result;
    getVariables(motor, offset, 1, &result);
    return result;
  }

  /// Reads two bytes from the Motoron using a "Get variables" command.
  ///
  /// \param motor 0 to read general variables, or a motor number to read
  ///   motor-specific variables.
  /// \param offset The location of the first byte to read.
  uint16_t getVar16(uint8_t motor, uint8_t offset)
  {
    uint8_t buffer[2];
    getVariables(motor, offset, 2, buffer);
    return buffer[0] | ((uint16_t)buffer[1] << 8);
  }

  /// Reads the "Status flags" variable from the Motoron.
  ///
  /// The bits in this variable are defined by the MOTORON_STATUS_FLAGS_*
  /// macros:
  ///
  /// - MOTORON_STATUS_FLAG_PROTOCOL_ERROR
  /// - MOTORON_STATUS_FLAG_CRC_ERROR
  /// - MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED
  /// - MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED
  /// - MOTORON_STATUS_FLAG_NO_POWER_LATCHED
  /// - MOTORON_STATUS_FLAG_RESET
  /// - MOTORON_STATUS_FLAG_COMMAND_TIMEOUT
  /// - MOTORON_STATUS_FLAG_MOTOR_FAULTING
  /// - MOTORON_STATUS_FLAG_NO_POWER
  /// - MOTORON_STATUS_FLAG_ERROR_ACTIVE
  /// - MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED
  /// - MOTORON_STATUS_FLAG_MOTOR_DRIVING
  ///
  /// Here is some example code that uses C++ bitwise operators to check
  /// whether there is currently a motor fault or a lack of power:
  ///
  /// ```{.cpp}
  /// uint16_t mask = (1 << MOTORON_STATUS_FLAG_NO_POWER) |
  ///   (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING);
  /// if (getStatusFlags() & mask) {  /* do something */ }
  /// ```
  ///
  /// This library has helper methods that make it easier if you just want to
  /// read a single bit:
  ///
  /// - getProtocolErrorFlag()
  /// - getCrcErrorFlag()
  /// - getCommandTimeoutLatchedFlag()
  /// - getMotorFaultLatchedFlag()
  /// - getNoPowerLatchedFlag()
  /// - getResetFlag()
  /// - getMotorFaultingFlag()
  /// - getNoPowerFlag()
  /// - getErrorActiveFlag()
  /// - getMotorOutputEnabledFlag()
  /// - getMotorDrivingFlag()
  ///
  /// The clearLatchedStatusFlags() method sets the specified set of latched
  /// status flags to 0.  The reinitialize() and reset() commands reset the
  /// latched status flags to their default values.
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  uint16_t getStatusFlags()
  {
    return getVar16(0, MOTORON_VAR_STATUS_FLAGS);
  }

  /// Returns the "Protocol error" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getProtocolErrorFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR);
  }

  /// Returns the "CRC error" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getCrcErrorFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_CRC_ERROR);
  }

  /// Returns the "Command timeout latched" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getCommandTimeoutLatchedFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED);
  }

  /// Returns the "Motor fault latched" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getMotorFaultLatchedFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED);
  }

  /// Returns the "No power latched" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getNoPowerLatchedFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED);
  }

  /// Returns the "Reset" bit from getStatusFlags().
  ///
  /// This bit is set to 1 when the Motoron powers on, its processor is
  /// reset (e.g. by reset()), or it receives a reinitialize() command.
  /// It can be cleared using clearResetFlag() or clearLatchedStatusFlags().
  ///
  /// By default, the Motoron is configured to treat this bit as an error,
  /// so you will need to clear it before you can turn on the motors.
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getResetFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_RESET);
  }

  /// Returns the "Motor faulting" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getMotorFaultingFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING);
  }

  /// Returns the "No power" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getNoPowerFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_NO_POWER);
  }

  /// Returns the "Error active" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getErrorActiveFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE);
  }

  /// Returns the "Motor output enabled" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getMotorOutputEnabledFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED);
  }

  /// Returns the "Motor driving" bit from getStatusFlags().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getMotorDrivingFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING);
  }

  /// Reads voltage on the Motoron's VIN pin, in raw device units.
  ///
  /// For more information, see the "VIN voltage" variable in the Motoron
  /// user's guide.
  ///
  /// \sa getVinVoltageMv()
  uint16_t getVinVoltage()
  {
    return getVar16(0, MOTORON_VAR_VIN_VOLTAGE);
  }

  /// Reads the voltage on the Motoron's VIN pin and converts it to millivolts.
  ///
  /// For more information, see the "VIN voltage" variable in the Motoron
  /// user's guide.
  ///
  /// \param referenceMv The reference voltage (IOREF), in millivolts.
  ///   For example, use 3300 for a 3.3 V system or 5000 for a 5 V system.
  ///
  /// \sa getVinVoltage()
  uint32_t getVinVoltageMv(uint16_t referenceMv)
  {
    return (uint32_t)getVinVoltage() * referenceMv / 1024 * 1047 / 47;
  }

  /// Reads the "Command timeout" variable and converts it to milliseconds.
  ///
  /// For more information, see the "Command timeout" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setCommandTimeoutMilliseconds()
  uint16_t getCommandTimeoutMilliseconds()
  {
    return getVar16(0, MOTORON_VAR_COMMAND_TIMEOUT) * 4;
  }

  /// Reads the "Error response" variable, which defines how the Motoron will
  /// stop its motors when an error is happening.
  ///
  /// For more information, see the "Error response" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setErrorResponse()
  uint8_t getErrorResponse()
  {
    return getVar8(0, MOTORON_VAR_ERROR_RESPONSE);
  }

  /// Reads the "Error mask" variable, which defines which status flags are
  /// considered to be errors.
  ///
  /// For more information, see the "Error mask" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setErrorMask()
  uint16_t getErrorMask()
  {
    return getVar16(0, MOTORON_VAR_ERROR_MASK);
  }

  /// Reads the "Jumper state" variable.
  ///
  /// For more information, see the "Jumper state" variable in the Motoron
  /// user's guide.
  uint8_t getJumperState()
  {
    return getVar8(0, MOTORON_VAR_JUMPER_STATE);
  }

  /// Reads the target speed of the specified motor, which is the speed at
  /// which the motor has been commanded to move.
  ///
  /// For more information, see the "Target speed" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeed(), setAllSpeeds(), setAllSpeedsUsingBuffers()
  int16_t getTargetSpeed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_TARGET_SPEED);
  }

  /// Reads the target brake amount for the specified motor.
  ///
  /// For more information, see the "Target speed" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setTargetBrakeAmount()
  uint16_t getTargetBrakeAmount(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_TARGET_BRAKE_AMOUNT);
  }

  /// Reads the current speed of the specified motor, which is the speed that
  /// the Motoron is currently trying to apply to the motor.
  ///
  /// For more information, see the "Target speed" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeedNow(), setAllSpeedsNow(), setAllSpeedsNowUsingBuffers()
  int16_t getCurrentSpeed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_CURRENT_SPEED);
  }

  /// Reads the buffered speed of the specified motor.
  ///
  /// For more information, see the "Buffered speed" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setBufferedSpeed(), setAllBufferedSpeeds()
  int16_t getBufferedSpeed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_BUFFERED_SPEED);
  }

  /// Reads the PWM mode of the specified motor.
  ///
  /// For more information, see the "PWM mode" variable in the Motoron
  /// user's guide.
  ///
  /// \sa setPwmMode()
  uint8_t getPwmMode(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_PWM_MODE);
  }

  /// Reads the maximum acceleration of the specified motor for the forward
  /// direction.
  ///
  /// For more information, see the "Max acceleration forward" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxAcceleration(), setMaxAccelerationForward()
  uint16_t getMaxAccelerationForward(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_ACCEL_FORWARD);
  }

  /// Reads the maximum acceleration of the specified motor for the reverse
  /// direction.
  ///
  /// For more information, see the "Max acceleration reverse" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxAcceleration(), setMaxAccelerationReverse()
  uint16_t getMaxAccelerationReverse(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_ACCEL_REVERSE);
  }

  /// Reads the maximum deceleration of the specified motor for the forward
  /// direction.
  ///
  /// For more information, see the "Max deceleration forward" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxDeceleration(), setMaxDecelerationForward()
  uint16_t getMaxDecelerationForward(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_DECEL_FORWARD);
  }

  /// Reads the maximum deceleration of the specified motor for the reverse
  /// direction.
  ///
  /// For more information, see the "Max deceleration reverse" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxDeceleration(), setMaxDecelerationReverse()
  uint16_t getMaxDecelerationReverse(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_DECEL_REVERSE);
  }

  /// \cond

  // This function is used by Pololu for testing.
  uint16_t getMaxDecelerationTemporary(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_DECEL_TMP);
  }

  /// \endcond

  /// Reads the starting speed for the specified motor in the forward direction.
  ///
  /// For more information, see the "Starting speed forward" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setStartingSpeed(), setStartingSpeedForward()
  uint16_t getStartingSpeedForward(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_STARTING_SPEED_FORWARD);
  }

  /// Reads the starting speed for the specified motor in the reverse direction.
  ///
  /// For more information, see the "Starting speed reverse" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setStartingSpeed(), setStartingSpeedReverse()
  uint16_t getStartingSpeedReverse(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_STARTING_SPEED_REVERSE);
  }

  /// Reads the direction change delay for the specified motor in the
  /// forward direction.
  ///
  /// For more information, see the "Direction change delay forward" variable
  /// in the Motoron user's guide.
  ///
  /// \sa setDirectionChangeDelay(), setDirectionChangeDelayForward()
  uint8_t getDirectionChangeDelayForward(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_FORWARD);
  }

  /// Reads the direction change delay for the specified motor in the
  /// reverse direction.
  ///
  /// For more information, see the "Direction change delay reverse" variable
  /// in the Motoron user's guide.
  ///
  /// \sa setDirectionChangeDelay(), setDirectionChangeDelayReverse()
  uint8_t getDirectionChangeDelayReverse(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_REVERSE);
  }

  /// Reads the current limit for the specified motor.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// For more information, see the "Current limit" variable
  /// in the Motoron user's guide.
  ///
  /// \sa setCurrentLimit()
  uint16_t getCurrentLimit(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_CURRENT_LIMIT);
  }

  /// Reads all the results from the last current sense measurement for the
  /// specified motor.
  ///
  /// This function reads the "Current sense raw", "Current sense speed", and
  /// "Current sense processed" variables from the Motoron using a single
  /// command, so the values returned are all guaranteed to be part of the
  /// same measurement.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// \sa getCurrentSenseRawAndSpeed(), getCurrentSenseProcessedAndSpeed()
  MotoronCurrentSenseReading getCurrentSenseReading(uint8_t motor)
  {
    uint8_t buffer[6];
    getVariables(motor, MOTORON_MVAR_CURRENT_SENSE_RAW, sizeof(buffer), buffer);
    MotoronCurrentSenseReading r = {};
    r.raw = buffer[0] | ((uint16_t)buffer[1] << 8);
    r.speed = buffer[2] | ((uint16_t)buffer[3] << 8);
    r.processed = buffer[4] | ((uint16_t)buffer[5] << 8);
    return r;
  }

  /// This is like getCurrentSenseReading() but it only reads the raw current
  /// sense measurement and the speed.
  ///
  /// The 'processed' member of the returned structure will be 0.
  ///
  /// This only works for the high-power Motorons.
  MotoronCurrentSenseReading getCurrentSenseRawAndSpeed(uint8_t motor)
  {
    uint8_t buffer[4];
    getVariables(motor, MOTORON_MVAR_CURRENT_SENSE_RAW, sizeof(buffer), buffer);
    MotoronCurrentSenseReading r = {};
    r.raw = buffer[0] | ((uint16_t)buffer[1] << 8);
    r.speed = buffer[2] | ((uint16_t)buffer[3] << 8);
    return r;
  }

  /// This is like getCurrentSenseReading() but it only reads the processed
  /// current sense measurement and the speed.
  ///
  /// The 'raw' member of the returned structure will be 0.
  ///
  /// This only works for the high-power Motorons.
  MotoronCurrentSenseReading getCurrentSenseProcessedAndSpeed(uint8_t motor)
  {
    uint8_t buffer[4];
    getVariables(motor, MOTORON_MVAR_CURRENT_SENSE_SPEED, sizeof(buffer), buffer);
    MotoronCurrentSenseReading r = {};
    r.speed = buffer[0] | ((uint16_t)buffer[1] << 8);
    r.processed = buffer[2] | ((uint16_t)buffer[3] << 8);
    return r;
  }

  /// Reads the raw current sense measurement for the specified motor.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// For more information, see the "Current sense raw" variable
  /// in the Motoron user's guide.
  ///
  /// \sa getCurrentSenseReading()
  uint16_t getCurrentSenseRaw(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_CURRENT_SENSE_RAW);
  }

  /// Reads the processed current sense reading for the specified motor.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// The units of this reading depend on the logic voltage of the Motoron
  /// and on the specific model of Motoron that you have, and you can use
  /// MotoronI2C::currentSenseUnitsMilliamps() to calculate the units.
  ///
  /// The accuracy of this reading can be improved by measuring the current
  /// sense offset and setting it with setCurrentSenseOffset().
  /// See the "Current sense processed" variable in the Motoron user's guide for
  /// or the CurrentSenseCalibrate example for more information.
  ///
  /// Note that this reading will be 0xFFFF if an overflow happens during the
  /// calculation due to very high current.
  ///
  /// \sa getCurrentSenseProcessedAndSpeed()
  uint16_t getCurrentSenseProcessed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_CURRENT_SENSE_PROCESSED);
  }

  /// Reads the current sense offset setting.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// For more information, see the "Current sense offset" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setCurrentSenseOffset()
  uint8_t getCurrentSenseOffset(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_CURRENT_SENSE_OFFSET);
  }

  /// Reads the current sense minimum divisor setting and returns it as a speed
  /// between 0 and 800.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// For more information, see the "Current sense minimum divisor" variable in
  /// the Motoron user's guide.
  ///
  /// \sa setCurrentSenseMinimumDivisor()
  uint16_t getCurrentSenseMinimumDivisor(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_CURRENT_SENSE_MINIMUM_DIVISOR) << 2;
  }

  /// Configures the Motoron using a "Set variable" command.
  ///
  /// This library has helper methods to set every variable, so you should
  /// not need to call this function directly.
  ///
  /// \param motor 0 to set a general variable, or a motor number to set
  ///   motor-specific variables.
  /// \param offset The address of the variable to set (only certain offsets
  ///   are allowed).
  /// \param value The value to set the variable to.
  ///
  /// \sa getVariables()
  void setVariable(uint8_t motor, uint8_t offset, uint16_t value)
  {
    if (value > 0x3FFF) { value = 0x3FFF; }
    uint8_t cmd[] = {
      MOTORON_CMD_SET_VARIABLE,
      (uint8_t)(motor & 0x1F),
      (uint8_t)(offset & 0x7F),
      (uint8_t)(value & 0x7F),
      (uint8_t)((value >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets the command timeout period, in milliseconds.
  ///
  /// For more information, see the "Command timeout" variable
  /// in the Motoron user's guide.
  ///
  /// \sa disableCommandTimeout(), getCommandTimeoutMilliseconds()
  void setCommandTimeoutMilliseconds(uint16_t ms)
  {
    // Divide by 4, but round up, and make sure we don't have
    // an overflow if 0xFFFF is passed.
    uint16_t timeout = (ms / 4) + ((ms & 3) ? 1 : 0);
    setVariable(0, MOTORON_VAR_COMMAND_TIMEOUT, timeout);
  }

  /// Sets the error response, which defines how the Motoron will
  /// stop its motors when an error is happening.
  ///
  /// The response parameter should be one of:
  ///
  /// - MOTORON_ERROR_RESPONSE_COAST
  /// - MOTORON_ERROR_RESPONSE_BRAKE
  /// - MOTORON_ERROR_RESPONSE_COAST_NOW
  /// - MOTORON_ERROR_RESPONSE_BRAKE_NOW
  ///
  /// For more information, see the "Error response" variable in the Motoron
  /// user's guide.
  ///
  /// \sa getErrorResponse()
  void setErrorResponse(uint8_t response)
  {
    setVariable(0, MOTORON_VAR_ERROR_RESPONSE, response);
  }

  /// Sets the "Error mask" variable, which defines which status flags are
  /// considered to be errors.
  ///
  /// For more information, see the "Error mask" variable in the Motoron
  /// user's guide.
  ///
  /// \sa getErrorMask(), getStatusFlags()
  void setErrorMask(uint16_t mask)
  {
    setVariable(0, MOTORON_VAR_ERROR_MASK, mask);
  }

  /// This disables the Motoron's command timeout feature by resetting the
  /// the "Error mask" variable to its default value but with the command
  /// timeout bit cleared.
  ///
  /// By default, the Motoron's command timeout will occur if no valid commands
  /// are received in 1500 milliseconds, and the command timeout is treated as
  /// an error, so the motors will shut down.  You can use this function if you
  /// want to disable that feature.
  ///
  /// Note that this function overrides any previous values you set in the
  /// "Error mask" variable, so if you are using setErrorMask() in your program
  /// to configure which status flags are treated as errors, you do not need to
  /// use this function and you probably should not use this function.
  ///
  /// \sa setCommandTimeoutMilliseconds(), setErrorMask()
  void disableCommandTimeout()
  {
    setErrorMask(defaultErrorMask & ~(1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT));
  }

  /// Sets the PWM mode for the specified motor.
  ///
  /// The mode parameter should be one of the following:
  ///
  /// - MOTORON_PWM_MODE_DEFAULT (20 kHz)
  /// - MOTORON_PWM_MODE_1_KHZ 1
  /// - MOTORON_PWM_MODE_2_KHZ 2
  /// - MOTORON_PWM_MODE_4_KHZ 3
  /// - MOTORON_PWM_MODE_5_KHZ 4
  /// - MOTORON_PWM_MODE_10_KHZ 5
  /// - MOTORON_PWM_MODE_20_KHZ 6
  /// - MOTORON_PWM_MODE_40_KHZ 7
  /// - MOTORON_PWM_MODE_80_KHZ 8
  ///
  /// For more information, see the "PWM mode" variable in the Motoron user's
  /// guide.
  ///
  /// \sa getPwmMode()
  void setPwmMode(uint8_t motor, uint8_t mode)
  {
    setVariable(motor, MOTORON_MVAR_PWM_MODE, mode);
  }

  /// Sets the maximum acceleration of the specified motor for the forward
  /// direction.
  ///
  /// For more information, see the "Max acceleration forward" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxAcceleration(), getMaxAccelerationForward()
  void setMaxAccelerationForward(uint8_t motor, uint16_t accel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_ACCEL_FORWARD, accel);
  }

  /// Sets the maximum acceleration of the specified motor for the reverse
  /// direction.
  ///
  /// For more information, see the "Max acceleration reverse" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxAcceleration(), getMaxAccelerationReverse()
  void setMaxAccelerationReverse(uint8_t motor, uint16_t accel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_ACCEL_REVERSE, accel);
  }

  /// Sets the maximum acceleration of the specified motor (both directions).
  ///
  /// If this function succeeds, it is equivalent to calling
  /// setMaxAccelerationForward() and setMaxAccelerationReverse().
  void setMaxAcceleration(uint8_t motor, uint16_t accel)
  {
    setMaxAccelerationForward(motor, accel);
    if (getLastError()) { return; }
    setMaxAccelerationReverse(motor, accel);
  }

  /// Sets the maximum deceleration of the specified motor for the forward
  /// direction.
  ///
  /// For more information, see the "Max deceleration forward" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxDeceleration(), getMaxDecelerationForward()
  void setMaxDecelerationForward(uint8_t motor, uint16_t decel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_DECEL_FORWARD, decel);
  }

  /// Sets the maximum deceleration of the specified motor for the reverse
  /// direction.
  ///
  /// For more information, see the "Max deceleration reverse" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setMaxDeceleration(), getMaxDecelerationReverse()
  void setMaxDecelerationReverse(uint8_t motor, uint16_t decel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_DECEL_REVERSE, decel);
  }

  /// Sets the maximum deceleration of the specified motor (both directions).
  ///
  /// If this function succeeds, it is equivalent to calling
  /// setMaxDecelerationForward() and setMaxDecelerationReverse().
  void setMaxDeceleration(uint8_t motor, uint16_t decel)
  {
    setMaxDecelerationForward(motor, decel);
    if (getLastError()) { return; }
    setMaxDecelerationReverse(motor, decel);
  }

  /// Sets the starting speed of the specified motor for the forward
  /// direction.
  ///
  /// For more information, see the "Starting speed forward" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setStartingSpeed(), getStartingSpeedForward()
  void setStartingSpeedForward(uint8_t motor, uint16_t speed)
  {
    setVariable(motor, MOTORON_MVAR_STARTING_SPEED_FORWARD, speed);
  }

  /// Sets the starting speed of the specified motor for the reverse
  /// direction.
  ///
  /// For more information, see the "Starting speed reverse" variable in the
  /// Motoron user's guide.
  ///
  /// \sa setStartingSpeed(), getStartingSpeedReverse()
  void setStartingSpeedReverse(uint8_t motor, uint16_t speed)
  {
    setVariable(motor, MOTORON_MVAR_STARTING_SPEED_REVERSE, speed);
  }

  /// Sets the starting speed of the specified motor (both directions).
  ///
  /// If this function succeeds, it is equivalent to calling
  /// setStartingSpeedForward() and setStartingSpeedReverse().
  void setStartingSpeed(uint8_t motor, uint16_t speed)
  {
    setStartingSpeedForward(motor, speed);
    if (getLastError()) { return; }
    setStartingSpeedReverse(motor, speed);
  }

  /// Sets the direction change delay of the specified motor for the forward
  /// direction, in units of 10 ms.
  ///
  /// For more information, see the "Direction change delay forward" variable
  /// in the Motoron user's guide.
  ///
  /// \sa setDirectionChangeDelay(), getDirectionChangeDelayForward()
  void setDirectionChangeDelayForward(uint8_t motor, uint8_t duration)
  {
    setVariable(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_FORWARD, duration);
  }

  /// Sets the direction change delay of the specified motor for the reverse
  /// direction, in units of 10 ms.
  ///
  /// For more information, see the "Direction change delay reverse" variable
  /// in the Motoron user's guide.
  ///
  /// \sa setDirectionChangeDelay(), getDirectionChangeDelayReverse()
  void setDirectionChangeDelayReverse(uint8_t motor, uint8_t duration)
  {
    setVariable(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_REVERSE, duration);
  }

  /// Sets the direction change delay of the specified motor (both directions),
  /// in units of 10 ms.
  ///
  /// If this function succeeds, it is equivalent to calling
  /// setDirectionChangeDelayForward() and setDirectionChangeDelayReverse().
  void setDirectionChangeDelay(uint8_t motor, uint8_t duration)
  {
    setDirectionChangeDelayForward(motor, duration);
    if (getLastError()) { return; }
    setDirectionChangeDelayReverse(motor, duration);
  }

  /// Sets the current limit for the specified motor.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// The units of the current limit depend on the type of Motoron you have
  /// and the logic voltage of your system.  See the "Current limit" variable
  /// in the Motoron user's guide for more information, or see
  /// MotoronI2C::calculateCurrentLimit().
  ///
  /// \sa getCurrentLimit()
  void setCurrentLimit(uint8_t motor, uint16_t limit)
  {
    setVariable(motor, MOTORON_MVAR_CURRENT_LIMIT, limit);
  }

  /// Sets the current sense offset setting for the specified motor.
  ///
  /// This is one of the settings that determines how current sense
  /// readings are processed.  It is supposed to be the value returned by
  /// getCurrentSenseRaw() when Motor power is supplied to the Motoron and
  /// it is driving its motor outputs at speed 0.
  ///
  /// The CurrentSenseCalibrate example shows how to measure the current
  /// sense offsets and load them onto the Motoron using this function.
  ///
  /// If you do not care about measuring motor current, you do not need to
  /// set this variable.
  ///
  /// For more information, see the "Current sense offset" variable in the
  /// Motoron user's guide.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// \sa getCurrentSenseOffset()
  void setCurrentSenseOffset(uint8_t motor, uint8_t offset)
  {
    setVariable(motor, MOTORON_MVAR_CURRENT_SENSE_OFFSET, offset);
  }

  /// Sets the current sense minimum divisor setting for the specified motor,
  /// given a speed between 0 and 800.
  ///
  /// This is one of the settings that determines how current sense
  /// readings are processed.
  ///
  /// If you do not care about measuring motor current, you do not need to
  /// set this variable.
  ///
  /// For more information, see the "Current sense minimum divisor" variable in
  /// the Motoron user's guide.
  ///
  /// This only works for the high-power Motorons.
  ///
  /// \sa getCurrentSenseMinimumDivisor()
  void setCurrentSenseMinimumDivisor(uint8_t motor, uint16_t speed)
  {
    setVariable(motor, MOTORON_MVAR_CURRENT_SENSE_MINIMUM_DIVISOR, speed >> 2);
  }

  /// Sends a "Coast now" command to the Motoron, causing all of the motors to
  /// immediately start coasting.
  ///
  /// For more information, see the "Coast now" command in the Motoron
  /// user's guide.
  void coastNow()
  {
    uint8_t cmd = MOTORON_CMD_COAST_NOW;
    sendCommand(1, &cmd);
  }

  /// Sends a "Clear motor fault" command to the Motoron.
  ///
  /// If any of the Motoron's motors chips are currently experiencing a
  /// fault (error), or bit 0 of the flags argument is 1, this command makes
  /// the Motoron attempt to recover from the faults.
  ///
  /// For more information, see the "Clear motor fault" command in the Motoron
  /// user's guide.
  ///
  /// \sa clearMotorFaultUnconditional(), getMotorFaultingFlag()
  void clearMotorFault(uint8_t flags = 0)
  {
    uint8_t cmd[] = { MOTORON_CMD_CLEAR_MOTOR_FAULT, (uint8_t)(flags & 0x7F) };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sends a "Clear motor fault" command to the Motoron with the
  /// "unconditional" flag set, so the Motoron will attempt to recover
  /// from any motor faults even if no fault is currently occurring.
  ///
  /// This is a more robust version of clearMotorFault().
  void clearMotorFaultUnconditional()
  {
    clearMotorFault(1 << MOTORON_CLEAR_MOTOR_FAULT_UNCONDITIONAL);
  }

  /// Clears the specified flags in getStatusFlags().
  ///
  /// For each bit in the flags argument that is 1, this command clears the
  /// corresponding bit in the "Status flags" variable, setting it to 0.
  ///
  /// For more information, see the "Clear latched status flags" command in the
  /// Motoron user's guide.
  ///
  /// \sa getStatusFlags(), setLatchedStatusFlags()
  void clearLatchedStatusFlags(uint16_t flags)
  {
    uint8_t cmd[] = { MOTORON_CMD_CLEAR_LATCHED_STATUS_FLAGS,
      (uint8_t)(flags & 0x7F),
      (uint8_t)(flags >> 7 & 0x7F)
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Clears the Motoron's reset flag.
  ///
  /// The reset flag is a latched status flag in getStatusFlags() that is
  /// particularly important to clear: it gets set to 1 after the Motoron
  /// powers on or experiences a reset, and it is considered to be an error
  /// by default, so it prevents the motors from running.  Therefore, it is
  /// necessary to call this function (or clearLatchedStatusFlags()) to clear
  /// the Reset flag before you can get the motors running.

  /// We recommend that immediately after you clear the reset flag. you should
  /// configure the Motoron's motor settings and error response settings.
  /// That way, if the Motoron experiences an unexpected reset while your system
  /// is running, it will stop running its motors and it will not start them
  /// again until all the important settings have been configured.
  ///
  /// \sa clearLatchedStatusFlags()
  void clearResetFlag()
  {
    clearLatchedStatusFlags(1 << MOTORON_STATUS_FLAG_RESET);
  }

  /// Sets the specified flags in getStatusFlags().
  ///
  /// For each bit in the flags argument that is 1, this command sets the
  /// corresponding bit in the "Status flags" variable to 1.
  ///
  /// For more information, see the "Set latched status flags" command in the
  /// Motoron user's guide.
  ///
  /// \sa getStatusFlags(), setLatchedStatusFlags()
  void setLatchedStatusFlags(uint16_t flags)
  {
    uint8_t cmd[] = { MOTORON_CMD_SET_LATCHED_STATUS_FLAGS,
      (uint8_t)(flags & 0x7F),
      (uint8_t)(flags >> 7 & 0x7F)
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets the target speed of the specified motor.
  ///
  /// The current speed will start moving to the specified target speed,
  /// obeying any acceleration and deceleration limits.
  ///
  /// The motor number should be between 1 and the number of motors supported
  /// by the Motoron.
  ///
  /// The speed should be between -800 and 800.  Values outside that range
  /// will be clipped to -800 or 800 by the Motoron firmware.
  ///
  /// For more information, see the "Set speed" command in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeedNow(), setAllSpeeds()
  void setSpeed(uint8_t motor, int16_t speed)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_SPEED,
      (uint8_t)(motor & 0x7F),
      (uint8_t)(speed & 0x7F),
      (uint8_t)((speed >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets the target and current speed of the specified motor, ignoring
  /// any acceleration and deceleration limits.
  ///
  /// For more information, see the "Set speed" command in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeed(), setAllSpeedsNow()
  void setSpeedNow(uint8_t motor, int16_t speed)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_SPEED_NOW,
      (uint8_t)(motor & 0x7F),
      (uint8_t)(speed & 0x7F),
      (uint8_t)((speed >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets the buffered speed of the specified motor.
  ///
  /// This command does not immediately cause any change to the motor: it
  /// stores a speed for the specified motor in the Motoron so it can be
  /// used by later commands.
  ///
  /// For more information, see the "Set speed" command in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeed(), setAllBufferedSpeeds(),
  ///   setAllSpeedsUsingBuffers(), setAllSpeedsNowUsingBuffers()
  void setBufferedSpeed(uint8_t motor, int16_t speed)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_BUFFERED_SPEED,
      (uint8_t)(motor & 0x7F),
      (uint8_t)(speed & 0x7F),
      (uint8_t)((speed >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }


  /// Sets the target speeds of all the motors at the same time.
  ///
  /// The number of speed arguments you provide to this function must be equal
  /// to the number of motor channels your Motoron has, or else this command
  /// might not work.
  ///
  /// This is equivalent to calling setSpeed() once for each motor, but it is
  /// more efficient because all of the speeds are sent in the same command.
  ///
  /// For more information, see the "Set all speeds" command in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeed(), setAllSpeedsNow(), setAllBufferedSpeeds()
  void setAllSpeeds(int16_t speed1, int16_t speed2)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_ALL_SPEEDS,
      (uint8_t)(speed1 & 0x7F),
      (uint8_t)((speed1 >> 7) & 0x7F),
      (uint8_t)(speed2 & 0x7F),
      (uint8_t)((speed2 >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// An overload of setAllSpeeds() for Motorons with 3 channels.
  void setAllSpeeds(int16_t speed1, int16_t speed2, int16_t speed3)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_ALL_SPEEDS,
      (uint8_t)(speed1 & 0x7F),
      (uint8_t)((speed1 >> 7) & 0x7F),
      (uint8_t)(speed2 & 0x7F),
      (uint8_t)((speed2 >> 7) & 0x7F),
      (uint8_t)(speed3 & 0x7F),
      (uint8_t)((speed3 >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets the target and current speeds of all the motors at the same time.
  ///
  /// The number of speed arguments you provide to this function must be equal
  /// to the number of motor channels your Motoron has, or else this command
  /// might not work.
  ///
  /// This is equivalent to calling setSpeedNow() once for each motor, but it is
  /// more efficient because all of the speeds are sent in the same command.
  ///
  /// For more information, see the "Set all speeds" command in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeed(), setSpeedNow(), setAllSpeeds()
  void setAllSpeedsNow(int16_t speed1, int16_t speed2)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_ALL_SPEEDS_NOW,
      (uint8_t)(speed1 & 0x7F),
      (uint8_t)((speed1 >> 7) & 0x7F),
      (uint8_t)(speed2 & 0x7F),
      (uint8_t)((speed2 >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// An overload of setAllSpeedsNow() for Motorons with 3 channels.
  void setAllSpeedsNow(int16_t speed1, int16_t speed2, int16_t speed3)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_ALL_SPEEDS_NOW,
      (uint8_t)(speed1 & 0x7F),
      (uint8_t)((speed1 >> 7) & 0x7F),
      (uint8_t)(speed2 & 0x7F),
      (uint8_t)((speed2 >> 7) & 0x7F),
      (uint8_t)(speed3 & 0x7F),
      (uint8_t)((speed3 >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets the buffered speeds of all the motors.
  ///
  /// The number of speed arguments you provide to this function must be equal
  /// to the number of motor channels your Motoron has, or else this command
  /// might not work.
  ///
  /// This command does not immediately cause any change to the motors: it
  /// stores speed for each motor in the Motoron so they can be used by later
  /// commands.
  ///
  /// For more information, see the "Set all speeds" command in the Motoron
  /// user's guide.
  ///
  /// \sa setSpeed(), setBufferedSpeed(), setAllSpeeds(),
  ///   setAllSpeedsUsingBuffers(), setAllSpeedsNowUsingBuffers()
  void setAllBufferedSpeeds(int16_t speed1, int16_t speed2)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_ALL_BUFFERED_SPEEDS,
      (uint8_t)(speed1 & 0x7F),
      (uint8_t)((speed1 >> 7) & 0x7F),
      (uint8_t)(speed2 & 0x7F),
      (uint8_t)((speed2 >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// An overload of setAllBufferedSpeeds() for Motorons with 3 channels.
  void setAllBufferedSpeeds(int16_t speed1, int16_t speed2, int16_t speed3)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_ALL_BUFFERED_SPEEDS,
      (uint8_t)(speed1 & 0x7F),
      (uint8_t)((speed1 >> 7) & 0x7F),
      (uint8_t)(speed2 & 0x7F),
      (uint8_t)((speed2 >> 7) & 0x7F),
      (uint8_t)(speed3 & 0x7F),
      (uint8_t)((speed3 >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Sets each motor's target speed equal to the buffered speed.
  ///
  /// This command is the same as setAllSpeeds() except that the speeds are
  /// provided ahead of time using setBufferedSpeed() or setAllBufferedSpeeds().
  ///
  /// \sa setAllSpeedsNowUsingBuffers(), setBufferedSpeed(),
  ///   setAllBufferedSpeeds()
  void setAllSpeedsUsingBuffers()
  {
    uint8_t cmd = MOTORON_CMD_SET_ALL_SPEEDS_USING_BUFFERS;
    sendCommand(1, &cmd);
  }

  /// Sets each motor's target speed and current speed equal to the buffered
  /// speed.
  ///
  /// This command is the same as setAllSpeedsNow() except that the speeds are
  /// provided ahead of time using setBufferedSpeed() or setAllBufferedSpeeds().
  ///
  /// \sa setAllSpeedsUsingBuffers(), setBufferedSpeed(),
  ///   setAllBufferedSpeeds()
  void setAllSpeedsNowUsingBuffers()
  {
    uint8_t cmd = MOTORON_CMD_SET_ALL_SPEEDS_NOW_USING_BUFFERS;
    sendCommand(1, &cmd);
  }

  /// Commands the motor to brake, coast, or something in between.
  ///
  /// Sending this command causes the motor to decelerate to speed 0 obeying
  /// any relevant deceleration limits.  Once the current speed reaches 0, the
  /// motor will attempt to brake or coast as specified by this command, but
  /// due to hardware limitations it might not be able to.
  ///
  /// The motor number parameter should be between 1 and the number of motors
  /// supported by the Motoron.
  ///
  /// The amount parameter gets stored in the "Target brake amount" variable
  /// for the motor and should be between 0 (coasting) and 800 (braking).
  /// Values above 800 will be clipped to 800 by the Motoron firmware.
  ///
  /// See the "Set braking" command in the Motoron user's guide for more
  /// information.
  ///
  /// \sa setBrakingNow(), getTargetBrakeAmount()
  void setBraking(uint8_t motor, uint16_t amount)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_BRAKING,
      (uint8_t)(motor & 0x7F),
      (uint8_t)(amount & 0x7F),
      (uint8_t)((amount >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Commands the motor to brake, coast, or something in between.
  ///
  /// Sending this command causes the motor's current speed to change to 0.
  /// The motor will attempt to brake or coast as specified by this command,
  /// but due to hardware limitations it might not be able to.
  ///
  /// The motor number parameter should be between 1 and the number of motors
  /// supported by the Motoron.
  ///
  /// The amount parameter gets stored in the "Target brake amount" variable
  /// for the motor and should be between 0 (coasting) and 800 (braking).
  /// Values above 800 will be clipped to 800 by the Motoron firmware.
  ///
  /// See the "Set braking" command in the Motoron user's guide for more
  /// information.
  ///
  /// \sa setBraking(), getTargetBrakeAmount()
  void setBrakingNow(uint8_t motor, uint16_t amount)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_SET_BRAKING_NOW,
      (uint8_t)(motor & 0x7F),
      (uint8_t)(amount & 0x7F),
      (uint8_t)((amount >> 7) & 0x7F),
    };
    sendCommand(sizeof(cmd), cmd);
  }

  /// Resets the command timeout.
  ///
  /// This prevents the command timeout status flags from getting set for some
  /// time.  (The command timeout is also reset by every other Motoron command,
  /// as long as its parameters are valid.)
  ///
  /// For more information, see the "Reset command timeout" command in the
  /// Motoron user's guide.
  ///
  /// \sa disableCommandTimeout(), setCommandTimeoutMilliseconds()
  void resetCommandTimeout()
  {
    uint8_t cmd = MOTORON_CMD_RESET_COMMAND_TIMEOUT;
    sendCommand(1, &cmd);
  }

  /// This static method calculates the 7-bit CRC needed for a Motoron
  /// command or response.  Most users will not need to use this, since most
  /// methods in this library automatically append a CRC byte or check
  /// received CRC bytes when appropriate.
  static uint8_t calculateCrc(uint8_t length, const uint8_t * buffer,
    uint8_t init = 0)
  {
    uint8_t crc = init;
    for (uint8_t i = 0; i < length; i++)
    {
      crc = pgm_read_byte(&motoronCrcTable[crc ^ buffer[i]]);

      // The code below shows an alternative method to calculate the CRC
      // without using a lookup table.  It will generally be slower but save
      // program space.
      // crc ^= buffer[i];
      // for (uint8_t j = 0; j < 8; j++)
      // {
      //   if (crc & 1) { crc ^= 0x91; }
      //   crc >>= 1;
      // }
    }
    return crc;
  }

  /// Calculates a current limit value that can be passed to the Motoron
  /// using setCurrentLimit().
  ///
  /// \param milliamps The desired current limit, in units of mA.
  /// \param type Specifies what type of Motoron you are using.
  /// \param referenceMv The reference voltage (IOREF), in millivolts.
  ///   For example, use 3300 for a 3.3 V system or 5000 for a 5 V system.
  /// \param offset The offset of the raw current sense signal for the Motoron
  ///   channel.  This is the same measurement that you would put into the
  ///   Motoron's "Current sense offset" variable using setCurrentSenseOffset(),
  ///   so see the documentation of that function for more info.
  ///   The offset is typically 10 for 5 V systems and 15 for 3.3 V systems,
  ///   (50*1024/referenceMv) but it can vary widely.
  static uint16_t calculateCurrentLimit(uint32_t milliamps,
    MotoronCurrentSenseType type, uint16_t referenceMv, uint16_t offset)
  {
    if (milliamps > 1000000) { milliamps = 1000000; }
    uint16_t limit = (uint32_t)(offset * 125 + 64) / 128 +
      milliamps * 20 / (referenceMv * ((uint8_t)type & 3));
    if (limit > 1000) { limit = 1000; }
    return limit;
  }

  /// Calculates the units for the Motoron's current sense reading returned by
  /// getCurrentSenseProcessed(), in milliamps.
  ///
  /// To convert a reading from getCurrentSenseProcessed() to milliamps
  /// multiply it by the value returned from this function using 32-bit
  /// multiplication.  For example:
  ///
  ///     uint32_t ma = (uint32_t)processed * units;
  ///
  /// \param type Specifies what type of Motoron you are using.
  /// \param referenceMv The reference voltage (IOREF), in millivolts.
  ///   For example, use 3300 for a 3.3 V system or 5000 for a 5 V system.
  static constexpr uint16_t currentSenseUnitsMilliamps(
    MotoronCurrentSenseType type, uint16_t referenceMv)
  {
    return ((uint32_t)referenceMv * ((uint8_t)type & 3) * 25 / 512);
  }

protected:
  /// Zero if the last communication with the device was successful, non-zero
  /// otherwise.
  uint8_t lastError;

  /// See setProtocolOptions.
  uint8_t protocolOptions;

private:

  void sendCommand(uint8_t length, const uint8_t * cmd)
  {
    bool sendCrc = protocolOptions & (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS);
    sendCommandCore(length, cmd, sendCrc);
  }

  virtual void sendCommandCore(uint8_t length, const uint8_t * cmd, bool sendCrc) = 0;
  virtual void flushTransmission() = 0;
  virtual void readResponse(uint8_t length, uint8_t * response) = 0;

  void sendCommandAndReadResponse(uint8_t cmdLength, const uint8_t * cmd,
    uint8_t responseLength, uint8_t * response)
  {
    sendCommand(cmdLength, cmd);
    if (getLastError())
    {
      memset(response, 0, responseLength);
      return;
    }
    readResponse(responseLength, response);
  }

  const uint8_t defaultProtocolOptions =
    (1 << MOTORON_PROTOCOL_OPTION_I2C_GENERAL_CALL) |
    (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS) |
    (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES);

  const uint16_t defaultErrorMask =
    (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT) |
    (1 << MOTORON_STATUS_FLAG_RESET);
};

/// Represents an I2C connection to a Motoron Motor Controller.
class MotoronI2C : public MotoronBase
{
public:
  /// Creates a new MotoronI2C object to communicate with the Motoron over I2C.
  ///
  /// The `address` parameter specifies the 7-bit I2C address to use, and it
  /// must match the address that the Motoron is configured to use.
  MotoronI2C(uint8_t address = 16) : address(address) {}

  /// Configures this object to use the specified I2C bus.
  /// The default bus is Wire, which is typically the first or only I2C bus on
  /// an Arduino.  To use Wire1 instead, you can write:
  /// ```{.cpp}
  /// mc.setBus(&Wire1);
  /// ```
  /// \param bus A pointer to a TwoWire object representing the I2C bus to use.
  void setBus(TwoWire * bus)
  {
    this->bus = bus;
  }

  /// Returns a pointer to the I2C bus that this object is configured to
  /// use.
  TwoWire * getBus()
  {
    return this->bus;
  }

  /// Configures this object to use the specified 7-bit I2C address.
  /// This must match the address that the Motoron is configured to use.
  void setAddress(uint8_t address)
  {
    this->address = address;
  }

  /// Returns the 7-bit I2C address that this object is configured to use.
  uint8_t getAddress()
  {
    return address;
  }

private:
  TwoWire * bus = &Wire;
  uint8_t address;

  void sendCommandCore(uint8_t length, const uint8_t * cmd, bool sendCrc) override
  {
    bus->beginTransmission(address);
    for (uint8_t i = 0; i < length; i++)
    {
      bus->write(cmd[i]);
    }
    if (sendCrc)
    {
      bus->write(calculateCrc(length, cmd));
    }
    lastError = bus->endTransmission();
  }

  void flushTransmission() { }

  void readResponse(uint8_t length, uint8_t * response) override
  {
    bool crcEnabled = protocolOptions & (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES);
    uint8_t byteCount = bus->requestFrom(address, (uint8_t)(length + crcEnabled));
    if (byteCount != length + crcEnabled)
    {
      memset(response, 0, length);
      lastError = 50;
      return;
    }
    lastError = 0;
    uint8_t * ptr = response;
    for (uint8_t i = 0; i < length; i++)
    {
      *ptr = bus->read();
      ptr++;
    }
    if (crcEnabled && bus->read() != calculateCrc(length, response))
    {
      lastError = 51;
      return;
    }
  }
};

/// Represents a serial connection to a Motoron.
class MotoronSerial : public MotoronBase
{
public:
  /// Creates a new MotoronSerial object.
  ///
  /// The `deviceNumber` argument is optional.  If it is omitted or 255, the
  /// MotoronSerial object will use the compact protocol.  If it is a number
  /// between 0 and 127, the MotoronSerial object will use the Pololu protocol,
  /// and the argument must match the address that the
  /// Motoron is configured to use.
  ///
  /// After using this constructor, you must call setPort() to specify which
  /// serial port to use.
  MotoronSerial(uint8_t deviceNumber = 255) :
    deviceNumber(deviceNumber)
  {
  }

  /// Alternative constructor that allows you to specify a serial port,
  /// so you do not need to call setPort().
  MotoronSerial(Stream & port, uint8_t deviceNumber = 255) :
    port(&port),
    deviceNumber(deviceNumber)
  {
  }

  /// Configures this object to use the specified serial port object.
  ///
  /// If SERIAL_PORT_HARDWARE_OPEN is defined for your board, it is generally
  /// a good choice here.  For example:
  /// ```{.cpp}
  /// mc.setPort(&SERIAL_PORT_HARDWARE_OPEN);
  /// ```
  void setPort(Stream * port)
  {
    this->port = port;
  }

  /// Returns a pointer to the serial port that this object is configured to
  /// use.
  Stream * getPort()
  {
    return port;
  }

  /// Configures this object to use the specified device number.
  ///
  /// If the argument is 255, the
  /// MotoronSerial object will use the compact protocol.  If it is a number
  /// between 0 and 127, the MotoronSerial object will use the Pololu protocol,
  /// and the argument must match the address that the
  /// Motoron is configured to use.
  void setDeviceNumber(uint8_t deviceNumber)
  {
    this->deviceNumber = deviceNumber;
  }

  /// Gets the serial device number this object is configured to use.
  uint8_t getDeviceNumber()
  {
    return deviceNumber;
  }

private:
  Stream * port = nullptr;
  uint8_t deviceNumber = 255;

  void sendCommandCore(uint8_t length, const uint8_t * cmd, bool sendCrc) override
  {
    if (port == nullptr) { lastError = 52; return; }

    if (deviceNumber == 255)
    {
      port->write(cmd, length);
      if (sendCrc)
      {
        port->write(calculateCrc(length, cmd));
      }
    }
    else
    {
      uint8_t header[3] = {
        0xAA,
        (uint8_t)(deviceNumber & 0x7F),
        (uint8_t)(cmd[0] & 0x7F),
      };
      port->write(header, sizeof(header));
      port->write(cmd + 1, length - 1);
      if (sendCrc)
      {
        uint8_t crc = calculateCrc(sizeof(header), header);
        crc = calculateCrc(length - 1, cmd + 1, crc);
        port->write(crc);
      }
    }
    lastError = 0;
  }

  void flushTransmission()
  {
    port->flush();
  }

  void readResponse(uint8_t length, uint8_t * response) override
  {
    if (port == nullptr) { lastError = 52; return; }

    port->flush();

    size_t byteCount = port->readBytes(response, length);
    if (byteCount != length)
    {
      lastError = 50;
      memset(response, 0, length);
      return;
    }

    bool crcEnabled = protocolOptions & (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES);
    if (crcEnabled)
    {
      uint8_t crc = 0;
      if (port->readBytes(&crc, 1) != 1)
      {
        lastError = 49;
        return;
      }

      if (crc != calculateCrc(length, response))
      {
        lastError = 51;

        // tmphax
        Serial.print(F("RCRC [ "));
        for (uint8_t i = 0; i < length; i++)
        {
          Serial.print(response[i], HEX);
          Serial.print(' ');
        }
        Serial.print(']');
        Serial.print(' ');
        Serial.print(crc, HEX);
        Serial.print(' ');
        Serial.println(calculateCrc(length, response, 0), HEX);
        Serial.print(' ');
        Serial.println(calculateCrc(length, response, 0), HEX);

        return;
      }
    }
    lastError = 0;
  }

};

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

/// Represents an I2C connection to a Pololu Motoron Motor Controller.
class MotoronI2C
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
  /// \param bus A pointer to a TwoWire object reoresenting the I2C bus to use.
  void setBus(TwoWire * bus)
  {
    this->bus = bus;
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

  /// Returns 0 if the last communication with the device was successful, or
  /// a non-zero error code if there was an error.
  uint8_t getLastError()
  {
    return lastError;
  }

  /// Sends the "Get firmware version" command to get the device's firmware
  /// product ID and firmware version numbers.
  ///
  /// For more information, see the "Get firwmare version"
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
  /// The \p options argument should be 0 a combination of the following
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
  void reinitialize()
  {
    // Always send the reset command with a CRC byte to make it more reliable.
    uint8_t cmd = MOTORON_CMD_REINITIALIZE;
    sendCommandCore(1, &cmd, true);
    protocolOptions = defaultProtocolOptions;
  }

  /// Sends an "Update device number" command to the Motoron, which causes the
  /// Motoron to update the I2C address it is using.
  ///
  /// For more details, see the "Update device number" command in the
  /// Motoron user's guide.
  ///
  /// This function does not update what I2C address this library is configured
  /// to use.  You can do that by calling setAddress().
  void updateDeviceNumber()
  {
    uint8_t cmd = MOTORON_CMD_UPDATE_DEVICE_NUMBER;
    sendCommandCore(1, &cmd, true);
  }

  /// Reads information from the Motoron using a "Get variables" command.
  ///
  /// This library has helper methods to read every variable, but this method
  /// is useful if you want to get the raw bytes, or if you want to read
  /// multiple consecutive variables at the same time for efficiency.
  ///
  /// @param motor 0 to read general variables, or a motor number to read
  ///   motor-specific variables.
  /// @param offset The location of the first byte to read.
  /// @param length How many bytes to read.
  /// @param buffer A pointer to an array to store the bytes read
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
  /// @param motor 0 to read a general variable, or a motor number to read
  ///   a motor-specific variable.
  /// @param offset The location of the byte to read.
  uint8_t getVar8(uint8_t motor, uint8_t offset)
  {
    uint8_t result;
    getVariables(motor, offset, 1, &result);
    return result;
  }

  /// Reads two bytes from the Motoron using a "Get variables" command.
  ///
  /// @param motor 0 to read general variables, or a motor number to read
  ///   motor-specific variables.
  /// @param offset The location of the first byte to read.
  uint16_t getVar16(uint8_t motor, uint8_t offset)
  {
    uint8_t buffer[2];
    getVariables(motor, offset, 2, buffer);
    return buffer[0] | ((uint16_t)buffer[1] << 8);
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
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  uint16_t getStatusFlags()
  {
    return getVar16(0, MOTORON_VAR_STATUS_FLAGS);
  }

  /// Returns the "Protocol error" bit from getStatusFlags().
  ///
  /// This flag is set when the Motoron receives an invalid byte in a command
  /// other than the CRC byte.
  /// It can be cleared using clearLatchedStatusFlags() or reset().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getProtocolErrorFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR);
  }

  /// Returns the "CRC error" bit from getStatusFlags().
  ///
  /// This flag is set when the Motoron receives an invalid CRC byte
  /// in a command.
  /// It can be cleared using clearLatchedStatusFlags() or reset().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getCrcErrorFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_CRC_ERROR);
  }

  /// Returns the "Command timeout latched" bit from getStatusFlags().
  ///
  /// This flag is set when the Motoron's command timeout feature is activated
  /// because too much time has passed since it received a command.
  /// It can be cleared using clearLatchedStatusFlags() or reset().
  ///
  /// This is the latched version of getCommandTimeoutFlag().
  ///
  /// For more information, see the "Status flags" variable in the Motoron
  /// user's guide.
  bool getCommandTimeoutLatchedFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED);
  }

  bool getMotorFaultLatchedFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED);
  }

  bool getNoPowerLatchedFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED);
  }

  bool getResetFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_RESET);
  }

  bool getMotorFaultingFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING);
  }

  bool getNoPowerFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_NO_POWER);
  }

  bool getErrorActiveFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE);
  }

  bool getMotorOutputEnabledFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED);
  }

  bool getMotorDrivingFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING);
  }

  uint16_t getCommandTimeoutMilliseconds()
  {
    return getVar16(0, MOTORON_VAR_COMMAND_TIMEOUT) * 4;
  }

  uint8_t getErrorResponse()
  {
    return getVar8(0, MOTORON_VAR_ERROR_RESPONSE);
  }

  uint16_t getErrorMask()
  {
    return getVar16(0, MOTORON_VAR_ERROR_MASK);
  }

  uint8_t getJumperState()
  {
    return getVar16(0, MOTORON_VAR_JUMPER_STATE);
  }

  int16_t getTargetSpeed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_TARGET_SPEED);
  }

  uint16_t getTargetBrakeAmount(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_TARGET_BRAKE_AMOUNT);
  }

  // This function is used by Pololu for testing.
  // TODO: remove
  bool getCoastRequest(uint8_t motor)
  {
    return getTargetBrakeAmount(motor) == 0;
  }

  int16_t getCurrentSpeed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_CURRENT_SPEED);
  }

  int16_t getBufferedSpeed(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_BUFFERED_SPEED);
  }

  uint8_t getPwmMode(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_PWM_MODE);
  }

  uint16_t getMaxAccelerationForward(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_ACCEL_FORWARD);
  }

  uint16_t getMaxAccelerationReverse(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_ACCEL_REVERSE);
  }

  uint16_t getMaxDecelerationForward(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_DECEL_FORWARD);
  }

  uint16_t getMaxDecelerationReverse(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_DECEL_REVERSE);
  }

  // This function is used by Pololu for testing.
  uint16_t getMaxDecelerationTemporary(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_MAX_DECEL_TMP);
  }

  uint16_t getStartingSpeedForward(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_STARTING_SPEED_FORWARD);
  }

  uint16_t getStartingSpeedReverse(uint8_t motor)
  {
    return getVar16(motor, MOTORON_MVAR_STARTING_SPEED_REVERSE);
  }

  uint8_t getDirectionChangeDelayForward(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_FORWARD);
  }

  uint8_t getDirectionChangeDelayReverse(uint8_t motor)
  {
    return getVar8(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_REVERSE);
  }

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

  void setCommandTimeoutMilliseconds(uint16_t ms)
  {
    // Divide by 4, but round up, and make sure we don't have
    // an overflow if 0xFFFF is passed.
    uint16_t timeout = (ms / 4) + ((ms & 3) ? 1 : 0);
    setVariable(0, MOTORON_VAR_COMMAND_TIMEOUT, timeout);
  }

  void setErrorResponse(uint8_t response)
  {
    setVariable(0, MOTORON_VAR_ERROR_RESPONSE, response);
  }

  void setErrorMask(uint16_t mask)
  {
    setVariable(0, MOTORON_VAR_ERROR_MASK, mask);
  }

  void disableCommandTimeout()
  {
    setErrorMask(defaultErrorMask & ~(1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT));
  }

  void setPwmMode(uint8_t motor, uint8_t mode)
  {
    setVariable(motor, MOTORON_MVAR_PWM_MODE, mode);
  }

  void setMaxAccelerationForward(uint8_t motor, uint16_t accel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_ACCEL_FORWARD, accel);
  }

  void setMaxAccelerationReverse(uint8_t motor, uint16_t accel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_ACCEL_REVERSE, accel);
  }

  void setMaxAcceleration(uint8_t motor, uint16_t accel)
  {
    setMaxAccelerationForward(motor, accel);
    if (getLastError()) { return; }
    setMaxAccelerationReverse(motor, accel);
  }

  void setMaxDecelerationForward(uint8_t motor, uint16_t decel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_DECEL_FORWARD, decel);
  }

  void setMaxDecelerationReverse(uint8_t motor, uint16_t decel)
  {
    setVariable(motor, MOTORON_MVAR_MAX_DECEL_REVERSE, decel);
  }

  void setMaxDeceleration(uint8_t motor, uint16_t decel)
  {
    setMaxDecelerationForward(motor, decel);
    if (getLastError()) { return; }
    setMaxDecelerationReverse(motor, decel);
  }

  void setStartingSpeedForward(uint8_t motor, uint16_t speed)
  {
    setVariable(motor, MOTORON_MVAR_STARTING_SPEED_FORWARD, speed);
  }

  void setStartingSpeedReverse(uint8_t motor, uint16_t speed)
  {
    setVariable(motor, MOTORON_MVAR_STARTING_SPEED_REVERSE, speed);
  }

  void setStartingSpeed(uint8_t motor, uint16_t speed)
  {
    setStartingSpeedForward(motor, speed);
    if (getLastError()) { return; }
    setStartingSpeedReverse(motor, speed);
  }

  void setDirectionChangeDelayForward(uint8_t motor, uint8_t duration)
  {
    setVariable(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_FORWARD, duration);
  }

  void setDirectionChangeDelayReverse(uint8_t motor, uint8_t duration)
  {
    setVariable(motor, MOTORON_MVAR_DIRECTION_CHANGE_DELAY_REVERSE, duration);
  }

  void setDirectionChangeDelay(uint8_t motor, uint8_t duration)
  {
    setDirectionChangeDelayForward(motor, duration);
    if (getLastError()) { return; }
    setDirectionChangeDelayReverse(motor, duration);
  }

  void coastNow()
  {
    uint8_t cmd = MOTORON_CMD_COAST_NOW;
    sendCommand(1, &cmd);
  }

  void clearMotorFault(uint8_t flags = 0)
  {
    uint8_t cmd[] = { MOTORON_CMD_CLEAR_MOTOR_FAULT, (uint8_t)(flags & 0x7F) };
    sendCommand(sizeof(cmd), cmd);
  }

  void clearMotorFaultUnconditional()
  {
    clearMotorFault(1 << MOTORON_CLEAR_MOTOR_FAULT_UNCONDITIONAL);
  }

  void clearLatchedStatusFlags(uint16_t flags)
  {
    uint8_t cmd[] = { MOTORON_CMD_CLEAR_LATCHED_STATUS_FLAGS,
      (uint8_t)(flags & 0x7F),
      (uint8_t)(flags >> 7 & 0x7F)
    };
    sendCommand(sizeof(cmd), cmd);
  }

  void clearResetFlag()
  {
    clearLatchedStatusFlags(1 << MOTORON_STATUS_FLAG_RESET);
  }

  void setLatchedStatusFlags(uint16_t flags)
  {
    uint8_t cmd[] = { MOTORON_CMD_SET_LATCHED_STATUS_FLAGS,
      (uint8_t)(flags & 0x7F),
      (uint8_t)(flags >> 7 & 0x7F)
    };
    sendCommand(sizeof(cmd), cmd);
  }

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

  void setAllSpeedsUsingBuffers()
  {
    uint8_t cmd = MOTORON_CMD_SET_ALL_SPEEDS_USING_BUFFERS;
    sendCommand(1, &cmd);
  }

  void setAllSpeedsNowUsingBuffers()
  {
    uint8_t cmd = MOTORON_CMD_SET_ALL_SPEEDS_NOW_USING_BUFFERS;
    sendCommand(1, &cmd);
  }

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

  void resetCommandTimeout()
  {
    uint8_t cmd = MOTORON_CMD_RESET_COMMAND_TIMEOUT;
    sendCommand(1, &cmd);
  }

  static uint8_t calculateCrc(uint8_t length, const uint8_t * buffer)
  {
    uint8_t crc = 0;
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

private:

  void sendCommandCore(uint8_t length, const uint8_t * cmd, bool sendCrc)
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

  void sendCommand(uint8_t length, const uint8_t * cmd)
  {
    bool sendCrc = protocolOptions & (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS);
    sendCommandCore(length, cmd, sendCrc);
  }

  void readResponse(uint8_t length, uint8_t * response)
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

  TwoWire * bus = &Wire;
  uint8_t address;
  uint8_t lastError = 0;
  uint8_t protocolOptions = defaultProtocolOptions; // TODO: remove this and most of the helpers that use it?
};

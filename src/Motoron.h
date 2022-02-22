#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "motoron_protocol.h"

extern const PROGMEM uint8_t motoronCrcTable[256];

/// Represents an I2C connection to a Pololu Motoron Motor Controller.
class MotoronI2C
{
public:
  MotoronI2C(uint8_t address = 15) : address(address) {}

  void setBus(TwoWire * bus)
  {
    this->bus = bus;
  }

  uint8_t getAddress()
  {
    return address;
  }

  void setAddress(uint8_t address)
  {
    this->address = address;
  }

  uint8_t getLastError()
  {
    return lastError;
  }

  void getFirmwareVersion(uint16_t * productId, uint16_t * firmwareVersion)
  {
    uint8_t cmd = MOTORON_CMD_GET_FIRMWARE_VERSION;
    uint8_t response[4];
    sendCommandAndReadResponse(1, &cmd, sizeof(response), response);
    if (productId != nullptr) { *productId = response[0] | (response[1] << 8); }
    if (firmwareVersion != nullptr) { *firmwareVersion = response[2] | (response[3] << 8); }
  }

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

  void enableCrc()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS)
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  void disableCrc()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS)
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  void enableCrcForCommands()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS));
  }

  void disableCrcForCommands()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_COMMANDS));
  }

  void enableCrcForResponses()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  void disableCrcForResponses()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES));
  }

  void enableI2cGeneralCall()
  {
    setProtocolOptions(protocolOptions
      | (1 << MOTORON_PROTOCOL_OPTION_I2C_GENERAL_CALL));
  }

  void disableI2cGeneralCall()
  {
    setProtocolOptions(protocolOptions
      & ~(1 << MOTORON_PROTOCOL_OPTION_I2C_GENERAL_CALL));
  }

  void readEeprom(uint8_t offset, uint8_t length, uint8_t * buffer)
  {
    uint8_t cmd[] = {
      MOTORON_CMD_READ_EEPROM,
      (uint8_t)(offset & 0x7F),
      (uint8_t)(length & 0x7F),
    };
    sendCommandAndReadResponse(sizeof(cmd), cmd, length, buffer);
  }

  uint8_t readEepromDeviceNumber()
  {
    uint8_t number;
    readEeprom(MOTORON_SETTING_DEVICE_NUMBER, 1, &number);
    return number;
  }

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

  void writeEepromDeviceNumber(uint8_t number)
  {
    writeEeprom(MOTORON_SETTING_DEVICE_NUMBER, number);
  }

  void reset()
  {
    // Always send the reset command with a CRC byte to make it more reliable.
    uint8_t cmd = MOTORON_CMD_RESET;
    sendCommandCore(1, &cmd, true);
    protocolOptions = defaultProtocolOptions;
  }

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

  uint8_t getVar8(uint8_t motor, uint8_t offset)
  {
    uint8_t result;
    getVariables(motor, offset, 1, &result);
    return result;
  }

  uint16_t getVar16(uint8_t motor, uint8_t offset)
  {
    uint8_t buffer[2];
    getVariables(motor, offset, 2, buffer);
    return buffer[0] | ((uint16_t)buffer[1] << 8);
  }

  uint16_t getVinVoltage()
  {
    return getVar16(0, MOTORON_VAR_VIN_VOLTAGE);
  }

  uint32_t getVinVoltageMv(uint16_t referenceMv)
  {
    return (uint32_t)getVinVoltage() * referenceMv / 1024 * 1047 / 47;
  }

  uint16_t getStatusFlags()
  {
    return getVar16(0, MOTORON_VAR_STATUS_FLAGS);
  }

  bool getProtocolErrorFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR);
  }

  bool getCrcErrorFlag()
  {
    return getStatusFlags() & (1 << MOTORON_STATUS_FLAG_CRC_ERROR);
  }

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

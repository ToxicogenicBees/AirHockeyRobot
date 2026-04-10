// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/// \file HighPowerStepperDriver.h
///
/// This is the main header file for the HighPowerStepperDriver library,
/// a library for controlling Pololu's High-Power Stepper Motor Drivers that are
/// based on the DRV8711.
///
/// For more information about this library, see:
///
///   https://github.com/pololu/high-power-stepper-driver-arduino
///
/// That is the main repository for this library.

// ***********************************************************
// Polulu HighPowerStepperDriver library Edited for use with the new DRV8462 drivers by Design Team 06.
// ***********************************************************

#pragma once

#include <Arduino.h>
#include <SPI.h>

/// Addresses of control and status registers.
enum class HPSDRegAddr : uint8_t
{
  CTRL1   = 0x04,
  CTRL2   = 0x05,
  CTRL3   = 0x06,
  CTRL10 = 0x0D,
  CTRL11   = 0x0E,
  CTRL12   = 0x0F,
};

/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8462 stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
class DRV8462_SPI
{
public:
  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    csPin = pin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, LOW);
  }

  /// Reads the register at the given address and returns its raw value.
  uint8_t readReg(uint8_t address)
  {
    // Read/write bit and register address are the first 7 bits of the first
    // byte; data is in the second byte (data all zeros for read)

    selectChip();
    uint16_t dataOut = transfer((0x01000000 | (address & 0b111111)) << 8);
    deselectChip();
    return dataOut & 0xFF;
  }

  /// Reads the register at the given address and returns its raw value.
  uint8_t readReg(HPSDRegAddr address)
  {
    return readReg((uint8_t)address);
  }

  /// Writes the specified value to a register.
  void writeReg(uint8_t address, uint8_t value)
  {
    // Read/write bit and register address are the first 7 bits of the first
    // byte; data is in the second byte (data all zeros for read)

    selectChip();
    transfer(((0x00000000 | (address & 0b111111)) << 8) | (value & 0xFF));
    deselectChip();
  }

  /// Writes the specified value to a register.
  void writeReg(HPSDRegAddr address, uint8_t value)
  {
    writeReg((uint8_t)address, value);
  }

private:

  SPISettings settings = SPISettings(100000, MSBFIRST, SPI_MODE1);

  uint16_t transfer(uint16_t value)
  {
    return SPI.transfer16(value);
  }

  void selectChip()
  {
    digitalWrite(csPin, LOW);
    SPI.beginTransaction(settings);
  }

  void deselectChip()
  {
   SPI.endTransaction();
   digitalWrite(csPin, HIGH);
  }

  uint8_t csPin;
};

/// This class provides high-level functions for controlling a DRV8711-based
/// High-Power Stepper Motor Driver.
class HighPowerStepperDriver
{
public:
  /// The default constructor.
  HighPowerStepperDriver()
  {
    // // All settings set to power-on defaults
    // ctrl   = 0xC10;
    // torque = 0x1FF;
    // off    = 0x030;
    // blank  = 0x080;
    // decay  = 0x110;
    // stall  = 0x040;
    // drive  = 0xA59;
  }

  /// Configures this object to use the specified pin as a chip select pin.
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    driver.setChipSelectPin(pin);
  }

  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  // void resetSettings()
  // {
  //   ctrl   = 0xC10;
  //   torque = 0x1FF;
  //   off    = 0x030;
  //   blank  = 0x080;
  //   decay  = 0x110;
  //   stall  = 0x040;
  //   drive  = 0xA59;
  //   applySettings();
  // }

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  // /// they do not.
  // bool verifySettings()
  // {
  //   // Bit 10 in TORQUE is write-only and will always read as 0.
  //   return driver.readReg(HPSDRegAddr::CTRL)   == ctrl   &&
  //          driver.readReg(HPSDRegAddr::TORQUE) == (torque & ~(1 << 10)) &&
  //          driver.readReg(HPSDRegAddr::OFF)    == off    &&
  //          driver.readReg(HPSDRegAddr::BLANK)  == blank  &&
  //          driver.readReg(HPSDRegAddr::DECAY)  == decay  &&
  //          driver.readReg(HPSDRegAddr::STALL)  == stall  &&
  //          driver.readReg(HPSDRegAddr::DRIVE)  == drive;
  // }

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  // void applySettings()
  // {
  //   writeTORQUE();
  //   writeOFF();
  //   writeBLANK();
  //   writeDECAY();
  //   writeDRIVE();
  //   writeSTALL();

  //   // CTRL is written last because it contains the ENBL bit, and we want to try
  //   // to have all the other settings correct first.  (For example, TORQUE
  //   // defaults to 0xFF (the maximum value), so it would be better to set a more
  //   // appropriate value if necessary before enabling the motor.)
  //   writeCTRL();
  // }

  /// Enables the driver
  void enableDriver()
  {
    driver.writeReg(HPSDRegAddr::CTRL1, 0b10001111);
  }

  /// Disables the driver (ENBL = 0).
  void disableDriver()
  {
    driver.writeReg(HPSDRegAddr::CTRL1, 0b00001111);
  }

  /// Sets the motor direction (RDIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You can use this command to control the direction of the stepper motor and
  /// leave the DIR pin disconnected.
  void setDirection(bool value)
  {
    
  }

  /// Returns the cached value of the motor direction (RDIR).
  ///
  /// This does not perform any SPI communication with the driver.
  bool getDirection()
  {
    // dummy function
    return false;
  }

  /// Advances the indexer by one step (RSTEP = 1).
  ///
  /// You can use this command to step the stepper motor and leave the STEP pin
  /// disconnected.
  ///
  /// The driver automatically clears the RSTEP bit after it is written.
  void step()
  {
    //
  }

  /// Sets the driver's stepping mode, defaulted to 1/8 microstepping.
  void setStepMode()
  {
    driver.writeReg(HPSDRegAddr::CTRL2, 0b00000101);  // 1/8 setting is 0101
  }

  /// Sets the current limit for a High-Power Stepper Motor Driver 36v4.
  ///
  /// The argument to this function should be the desired current limit in
  /// milliamps.
  ///
  /// WARNING: The 36v4 can supply up to about 4 A per coil continuously;
  /// higher currents might be sustainable for short periods, but can eventually
  /// cause the MOSFETs to overheat, which could damage them.  See the driver's
  /// product page for more information.
  ///
  /// This function allows you to set a current limit of up to 8 A (8000 mA),
  /// but we strongly recommend against using a current limit higher than 4 A
  /// (4000 mA) unless you are careful to monitor the MOSFETs' temperatures
  /// and/or restrict how long the driver uses the higher current limit.
  ///
  /// This function takes care of setting appropriate values for ISGAIN and
  /// TORQUE to get the desired current limit.
  void setCurrent(uint8_t current)
  {
    driver.writeReg(HPSDRegAddr::CTRL11, current);
  }

  void setCurrentHold(uint8_t current)
  {
    driver.writeReg(HPSDRegAddr::CTRL10, current);
  }

  void enableStandstillPowerSavingMode()
  {
    driver.writeReg(HPSDRegAddr::CTRL12, 0b10100000);
  }

  /// Sets the driver's decay mode (DECMOD).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(HPSDDecayMode::AutoMixed);
  /// ~~~
  void setDecayMode()
  {
    //
  }

  /// Reads the status of the driver (STATUS register).
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// status condition (the upper 4 bits of the 12-bit STATUS register are not
  /// used).  You can simply compare the return value to 0 to see if any of the
  /// status bits are set, or you can use the logical AND operator (`&`) and the
  /// #HPSDStatusBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readStatus() & (1 << (uint8_t)HPSDStatusBit::UVLO))
  /// {
  ///   // Undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readStatus()
  {
    // return driver.readReg(HPSDRegAddr::STATUS);

    // dummy function
    return 0;
  }

public:
  /// This object handles all the communication with the DRV8711.  Generally,
  /// you should not need to use it in your code for basic usage of a
  /// High-Power Stepper Motor Driver, but you might want to use it to access
  /// more advanced settings that the HighPowerStepperDriver class does not
  /// provide functions for.
  DRV8462_SPI driver;
};

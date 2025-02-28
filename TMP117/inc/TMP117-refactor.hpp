#pragma once

#include <cstdint>
#include <etl/expected.h>
#include <etl/optional.h>

#include "main.h"

using u8 = uint8_t;
using u16 = uint16_t;

using i16 = int16_t;

namespace TMP117_Refactor {
    /**
        The device I2C addresses (Page 21 TMP117 manual)
    */
    enum class I2CAddress : u8 {
        Address1 = 0x90,
        Address2 = 0x92,
        Address3 = 0x94,
        Address4 = 0x96
    };

    enum class RegisterAddress : u8 {
        TemperatureRegister = 0x00,
        ConfigurationRegister = 0x01,
        TemperatureHighLimit = 0x02,
        TemperatureLowLimit = 0x03,
        EEPROMUnlock = 0x04,
        EEPROM1 = 0x05,
        EEPROM2 = 0x06,
        TemperatureOffset = 0x07,
        EEPROM3 = 0x08,
        IDRegister = 0x0F
    };

    enum class Averaging : u8 {
        NoAveraging,
        Samples8,
        Samples16,
        Samples32
    };

    enum class ConversionCycle : u8 {
        Fastest,
        Default,
        MS250,
        MS500,
        MS1000,
        Low,
        Lower,
        Lowest
    };

    enum class ConversionMode : u8 {
        Continuous,
        ShutDown,
        ContinuousRed,
        OneShot
    };

    enum class ThermalAlert : u8 {
        AlertMode,
        ThermMode
    };

    enum class AlertPolarity : u8 {
        ActiveLow,
        ActiveHigh
    };

    enum class DRAlert : u8 {
        Alert,
        DataReady
    };

    enum class Error : u8 {
        NoError,
        Timeout,
        InvalidEEPROM,
        NoDataReady,
        InvalidCalibrationOffset,
        InvalidHighLimit,
        InvalidLowLimit
    };


    struct Config {
        float calibrationOffset;
        float highLimit;
        float lowLimit;
        ConversionMode conversionMode;
        ConversionCycle conversionCycle;
        Averaging averaging;
        ThermalAlert thermalAlert;
        AlertPolarity alertPolarity;
        DRAlert drAlert;
    };

    class TMP117 {
    public:
        static constexpr float TemperaturePrecision = 0.0078125;
        static constexpr u16 MaxTimeoutDelay = 100;
        static constexpr u16 MaxAbsoluteTemperature = 256;
        static constexpr u16 DeviceID = 0x117;
        static constexpr u8 RevisionNumber = 0;
    private:
        I2C_HandleTypeDef* hi2c1;
        Config config;
        I2CAddress i2cSlaveAddress;
    public:
        /**
         * Factory that constructs the TMP117 object and also performs a setup based on \p config
         *
         * @param hi2c1         I2C definition
         * @param address       Address of device in the I2C bus
         * @param config        Used for setting up the configuration register
         *
         * @return The created object if configuration succeeds or an error
         */
        static etl::expected<TMP117, Error> CreateTMP117(I2C_HandleTypeDef* hi2c1, const Config &config, I2CAddress address);
    private:
        TMP117(I2C_HandleTypeDef* hi2c1, const Config &config, const I2CAddress address)
            :
            hi2c1(hi2c1), config(config), i2cSlaveAddress(address)
        {}
    public:
        /**
         * Gets the temperature of the sensor.
         *
         * @note This directly reads from the configuration registers and clears any alert flag
         * @note Will perform a One-Shot conversion if called while in Shut-Down mode
         *
         * @return The temperature in Celsius or an error.
         */
        [[nodiscard]] etl::expected<float, Error> getTemperature() const;
    private:
        /**
         * Reprogram configuration register according to the given configuration.
         * Needs to be re-run each time configuration is changed for changes to be applied.
         *
         * @warning Avoid directly reading from the configuration register since alerts will be automatically cleared
         *
         * @return Possible raised error
         */
        [[nodiscard]] etl::expected<void, Error> setConfiguration() const;

        /**
         * Reads a register with I2C
         *
         * @param addr Slave Device Address
         *
         * @return The 16-bit value of the register or an error
         */
        [[nodiscard]] etl::expected<u16, Error> readRegister(RegisterAddress addr) const;

        /**
         * Writes a register with I2C
         *
         * @param addr Address of register to be read from
         * @param data Data to write to register
         * @return Possible raised error
         */
        [[nodiscard]] etl::expected<void, Error> writeRegister(RegisterAddress addr, u16 data) const;

        /**
         * Writes to the chip programmable memory.
         * @param addr Address of the EEPROM register
         * @param data Data to store to target memory
         *
         * @return Possible raised error
         */
        [[nodiscard]] etl::expected<void, Error> writeEEPROM(RegisterAddress addr, u16 data) const;

        [[nodiscard]] etl::expected<bool, Error> isBusyEEPROM() const;
        [[nodiscard]] etl::expected<void, Error> softReset() const;
        [[nodiscard]] etl::expected<bool, Error> isDataReady() const;
    };

    /**
     * Converts the temperature from a 16bit value into a float representing a temperature in the range -+ 256 [C]
     * @param temp  16bit temperature value
     * @return      Formatted value
     */
    static float convertToTemperature(u16 temp);


    /**
     * Converts a float temperature into a 16bit value representing a temperature in the range -+ 256 [C]
     * @param temp  Temperature value as float
     * @return      Formatted value
     */
    etl::expected<u16, Error> convertFromTemperature(float temp);

    /**
     * Converts a HAL status code into a TMP117::Error
     *
     * @param status HAL status
     *
     * @return Corresponding TMP117::Error
     */
    static Error handleHALStatus(u8 status);
}

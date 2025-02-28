#include "TMP117-refactor.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <TMP117.hpp>

namespace TMP117_Refactor {
    etl::expected<TMP117, Error> TMP117::CreateTMP117(I2C_HandleTypeDef* hi2c1, const Config &config, I2CAddress address) {
        auto tmp117 = TMP117(hi2c1, config, address);
        if (const auto writeResult = tmp117.setConfiguration(); !writeResult) {
            return etl::unexpected(writeResult.error());
        }
        return tmp117;
    }

    etl::expected<float, Error> TMP117::getTemperature() const {
        if (const ConversionMode mode = config.conversionMode;
            mode == ConversionMode::Continuous || mode == ConversionMode::ContinuousRed
        ) {
            const auto readTemperatureResult =
                readRegister(RegisterAddress::TemperatureRegister);
            if (!readTemperatureResult) {
                return etl::unexpected(readTemperatureResult.error());
            }
            return convertToTemperature(readTemperatureResult.value()) + config.calibrationOffset;
        }

        const auto readConfigResult = readRegister(RegisterAddress::ConfigurationRegister);
        if (!readConfigResult) {
            return etl::unexpected(readConfigResult.error());
        }

        if (const auto writeResult = writeRegister(
            RegisterAddress::ConfigurationRegister,
            readConfigResult.value() & 0xC00
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        auto dataReadyResult = isDataReady();
        u8 msElapsed = 0;
        while (dataReadyResult && !dataReadyResult.value() && msElapsed < MaxTimeoutDelay) {
            #if defined(INC_FREERTOS_H)
                vTaskDelay(pdMS_TO_TICKS(7));
            #else
                HAL_Delay(7);
            #endif
            msElapsed += 7;
            dataReadyResult = isDataReady();
        }

        if (!dataReadyResult) {
            return etl::unexpected(dataReadyResult.error());
        }
        if (!dataReadyResult.value()) {
            return etl::unexpected(Error::Timeout);
        }

        const auto readTemperatureResult = readRegister(RegisterAddress::TemperatureRegister);
        if (!readTemperatureResult) {
            return etl::unexpected(readTemperatureResult.error());
        }
        return convertToTemperature(readTemperatureResult.value()) + config.calibrationOffset;
    }

    etl::expected<void, Error> TMP117::setConfiguration() const {
        const auto calibrationResult = convertFromTemperature(config.calibrationOffset);
        if (!calibrationResult) {
            return etl::unexpected(calibrationResult.error());
        }
        if (const auto writeResult = writeRegister(
            RegisterAddress::TemperatureOffset,
            calibrationResult.value()
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        const auto highLimitResult = convertFromTemperature(config.highLimit);
        if (!highLimitResult) {
            return etl::unexpected(highLimitResult.error());
        }
        if (const auto writeResult = writeRegister(
            RegisterAddress::TemperatureOffset,
            highLimitResult.value()
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        const auto lowLimitResult = convertFromTemperature(config.highLimit);
        if (!lowLimitResult) {
            return etl::unexpected(lowLimitResult.error());
        }
        if (const auto writeResult = writeRegister(
            RegisterAddress::TemperatureOffset,
            lowLimitResult.value()
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        const u16 configData =
            (static_cast<u16>(config.conversionMode) << 10) |
            (static_cast<u16>(config.conversionCycle) << 7) |
            (static_cast<u16>(config.averaging) << 5) |
            (static_cast<u16>(config.thermalAlert) << 4) |
            (static_cast<u16>(config.alertPolarity) << 3) |
            (static_cast<u16>(config.drAlert) << 2);

        if (const auto writeResult = writeRegister(
            RegisterAddress::ConfigurationRegister,
            configData
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }
        return {};
    }

    etl::expected<u16, Error> TMP117::readRegister(RegisterAddress addr) const {
        if (const Error err = handleHALStatus(
            HAL_I2C_Master_Transmit(
                hi2c1,
                static_cast<u16>(i2cSlaveAddress),
                reinterpret_cast<u8*>(&addr),
                1,
                MaxTimeoutDelay
            )
        ); err != Error::NoError) {
            return etl::unexpected(err);
        }

        u8 pData[2];
        if (const Error err = handleHALStatus(
            HAL_I2C_Master_Receive(
                hi2c1,
                static_cast<u16>(i2cSlaveAddress),
                pData,
                2,
                MaxTimeoutDelay
            )
        ); err != Error::NoError) {
            return etl::unexpected(err);
        }

        return (static_cast<u16>(pData[0]) >> 8) | static_cast<u16>(pData[1]);
    }

    etl::expected<void, Error> TMP117::writeRegister(const RegisterAddress addr, const u16 data) const {
        u8 pData[] = {
            static_cast<u8>(addr),
            static_cast<u8>((data >> 8) & 0xFF),
            static_cast<u8>(data & 0xFF)
        };

        if (const Error err = handleHALStatus(
            HAL_I2C_Master_Transmit(
                hi2c1,
                static_cast<u16>(i2cSlaveAddress),
                pData,
                3,
                MaxTimeoutDelay
            )
        ); err != Error::NoError) {
            return etl::unexpected(err);
        }
        return {};
    }

    etl::expected<void, Error> TMP117::writeEEPROM(const RegisterAddress addr, const u16 data) const {
        if (!(
            addr == RegisterAddress::EEPROM1 ||
            addr == RegisterAddress::EEPROM2 ||
            addr == RegisterAddress::EEPROM3
        )) {
            return etl::unexpected(Error::InvalidEEPROM);
        }

        if (const auto writeResult = writeRegister(
            RegisterAddress::EEPROMUnlock,
            0x8000
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        if (const auto writeResult = writeRegister(
            addr,
            data
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        auto busyResult = isBusyEEPROM();
        u8 msElapsed = 0;
        while (busyResult && busyResult.value() && msElapsed < MaxTimeoutDelay) {
            #if defined(INC_FREERTOS_H)
                vTaskDelay(pdMS_TO_TICKS(7));
            #else
                HAL_Delay(7);
            #endif
            msElapsed += 7;
            busyResult = isBusyEEPROM();
        }

        if (!busyResult) {
            return etl::unexpected(busyResult.error());
        }
        if (busyResult.value()) {
            return etl::unexpected(Error::Timeout);
        }

        //The datasheet states that an I2C general call reset is required in order to have the
        //values actually written to the EEPROM. However, such an action would affect every device
        //on the I2C bus, so a software reset is performed instead
        return softReset();
    }

    etl::expected<bool, Error> TMP117::isBusyEEPROM() const {
        const auto readUnlockResult = readRegister(RegisterAddress::EEPROMUnlock);
        if (!readUnlockResult) {
            return etl::unexpected(readUnlockResult.error());
        }
        return (readUnlockResult.value() & 0x4000) >> 14;
    }

    etl::expected<void, Error> TMP117::softReset() const {
        const auto readConfigResult = readRegister(RegisterAddress::ConfigurationRegister);
        if (!readConfigResult) {
            return etl::unexpected(readConfigResult.error());
        }
        if (const auto writeResult = writeRegister(
            RegisterAddress::ConfigurationRegister,
            readConfigResult.value() | 0x2
        ); !writeResult) {
            return etl::unexpected(writeResult.error());
        }

        //Wait for at least 2ms
        #if defined(INC_FREERTOS_H)
            vTaskDelay(pdMS_TO_TICKS(3));
        #else
            HAL_Delay(3);
        #endif

        //Lock the EEPROM
        const auto readUnlockResult = readRegister(RegisterAddress::EEPROMUnlock);
        if (!readUnlockResult) {
            return etl::unexpected(readUnlockResult.error());
        }
        return writeRegister(
            RegisterAddress::EEPROMUnlock,
            readUnlockResult.value() & 0x7FFF
        );
    }

    etl::expected<bool, Error> TMP117::isDataReady() const {
        const auto readConfigResult = readRegister(RegisterAddress::ConfigurationRegister);
        if (!readConfigResult) {
            return etl::unexpected(readConfigResult.error());
        }
        return readConfigResult.value() & 0x2000;
    }


    float convertToTemperature(const u16 temp) {
        return static_cast<float>(temp) * TMP117::TemperaturePrecision;
    }

    etl::expected<u16, Error> convertFromTemperature(const float temp) {
        if (temp > TMP117::MaxAbsoluteTemperature ||
            temp < -TMP117::MaxAbsoluteTemperature
        ) {
            return etl::unexpected(Error::InvalidCalibrationOffset);
        }

        if (temp >= 0) {
            return static_cast<i16>(temp / TMP117::TemperaturePrecision);
        }
        return static_cast<i16>(-temp / TMP117::TemperaturePrecision + 1);
    }

    Error handleHALStatus(const u8 status) {
        if (status == HAL_ERROR || status == HAL_BUSY || status == HAL_TIMEOUT) {
            return Error::Timeout;
        }
        return Error::NoError;
    }
}

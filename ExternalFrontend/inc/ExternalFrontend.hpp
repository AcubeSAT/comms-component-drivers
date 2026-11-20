#pragma once

#include <cstdint>
#include "main.h"
#include "etl/expected.h"
#include "etl/optional.h"
#include "etl/delegate.h"

namespace ExternalFrontend {
    enum class ErrorType: uint8_t {
        AD8318_TEMP_ADC_FAILURE,
        GAIN_ADC_FAILURE,
        SETPOINT_VOLTAGE_DAC_FAILURE,
        GAIN_OUT_OF_BOUNDS,
        UHF_RX_FRONT_END_DISABLED,
        NOT_IN_LOCKED_AGC_MODE,
        NOT_IN_UNLOCKED_AGC_MODE,
        INVALID_DAC_CHANNEL_NUMBER,
        INVALID_ADC_OR_DAC_RESOLUTION,
        IDENTICAL_CALIBRATION_POINTS,
        VOLTAGE_OUT_OF_BOUNDS,
        INVALID_CODE,
        NULL_PTR,
        INVALID_DELEGATE,
        INVALID_VREF
    };

    struct Error {
        ErrorType errorType;
        HAL_StatusTypeDef status;

        Error(ErrorType errorType, HAL_StatusTypeDef status) : errorType(errorType), status(status) {}
        Error(ErrorType errorType) : errorType(errorType), status(HAL_OK) {}
    };

    /**
     * A class that encapsulates all necessary functionality and state for the
     * external frontends of SatNOG COMMS v0.A3S board
     */
    class ExternalFrontendUtilities {
    public:
        ExternalFrontendUtilities() = default;

        /**
         * @brief Bring external frontend and used ADCs, DAC to an "off" state.
         *
         * @param ad8318_Vset1, ad8318_Vset2, ad8318_Pout1, ad8318_Pout2
         *
         * @param ad8318Temp_ADCHandle Handle to ADC that will be used to sample AD8318's temperature.
         *
         * @param gain_ADCHandle Handle to ADC that will be used to sample the current gain (AD8318 Vout). It must
         *        be a dedicated ADC, set to a fast sample rate and lower resolution, so locking of AGC
         *        may be performed in fewer cycles
         *
         * @param  vset_DACHandle, dac_ChannelNumber Handle to DAC and channel number (DAC_CHANNEL_1 OR DAC_CHANNEL_2) that will be used
         *          to set the target output power (AD8318 Vset)
         *
         * @param get_Vref A function that returns VREF+ (ADC and DAC reference voltage) when called
         *
         * @note This function does not configure the ADC and DAC peripherals. It is assumed that the HAL initialization
         *       code is run beforehand
         */
        [[nodiscard]] etl::expected<void, Error> initialize(
            ADC_HandleTypeDef* ad8318Temp_ADCHandle,
            ADC_HandleTypeDef* gain_ADCHandle,
            DAC_HandleTypeDef* vset_DACHandle,
            uint32_t dac_ChannelNumber,
            etl::delegate<float()> get_Vref,
            float ad8318_Vset1,
            float ad8318_Vset2,
            float ad8318_Pout1,
            float ad8318_Pout2);

        /// Flags that get set to true from a callback, in case the current limiters FLAGB pin goes
        ///  LOW, due to output overcurrent, input undervoltage or overheating.
        volatile bool currentLimiterAlertTxUHF = false;
        volatile bool currentLimiterAlertRxUHF = false;
        volatile bool currentLimiterAlertSBAND = false;

        /**
         * @brief Enable UHF Rx frontend load switch and set ADL5330 to a constant gain
         *
         * @param targetGainDB: The available gain values range between ADL5330MinGain and ADL5330MaxGain
         *
         */
        [[nodiscard]] etl::expected<void, Error> enableUhfRxFrontendConstantGain(float targetGainDB);

        /**
         * @brief Enable UHF Rx frontend load switch and set ADL5330 VGA to automatic gain control, by
         *        setting a target output power to AD8318 AGC
         */
        [[nodiscard]] etl::expected<void, Error> enableUhfRxFrontendAutomaticGainControl(float targetOutputPowerDBm);

        /**
         * @brief Enable UHF Tx frontend load switch
         */
        void enableUhfTxFrontend();

        /**
         * @brief Enable S-Band Tx frontend load switch
         */
        void enableSbandTxFrontend();

        /**
         * @brief Disable UHF Rx frontend load switch and ADCs, DAC
         */
        [[nodiscard]] etl::expected<void, Error> disableUhfRxFrontend();

        /**
         * @brief Disable UHF Tx frontend load switch
         */
        void disableUhfTxFrontend();

        /**
         * @brief Disable S-Band Tx frontend load switch
         */
        void disableSbandTxFrontend();

        /**
         * @brief Read the temperature from the integrated analog sensor of the AD8318 AGC.
         * @returns The temperature in Celsius, if the Rx-UHF chain is active and the ADC conversion succeeded.
         */
        [[nodiscard]] etl::expected<float, Error> readAGCTemperature();

        /**
         * @brief Switch AD8318 out of the loop and lock ADL5330 to the current gain
         */
        [[nodiscard]] etl::expected<void, Error> lockAGC();

        /**
         * @brief Switch AD8318 back in the loop with the previously stored target
         */
        [[nodiscard]] etl::expected<void, Error> unlockAGC();

        /**
         * @returns The current gain of the ADL5330 in dB, which in constant gain mode should be targetGainDb, as set
         *          by enableUhfRxFrontendConstantGain(). In automatic gain mode, it is equal to Vout of AD8318
         */
        [[nodiscard]] etl::expected<float, Error> readCurrentGain();

    private:
        enum class UHFRxFrontendState : uint8_t {
            OFF,
            CONSTANT_GAIN_MODE,
            AGC_MODE_LOCKED,
            AGC_MODE_UNLOCKED
        };

        /**
         * Route output of DAC directly to ADL5330
         */
        void switchToConstantGainMode() {
            HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_SET);
        }

        /**
         * Route output of DAC to AD8318
         */
        void switchToAGCMode() {
            HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_RESET);
        }

        /**
         * Convert ADC code to voltage (in mV)
         */
        [[nodiscard]] etl::expected<float, Error> adcCodeToVoltage(uint16_t adcCode, uint32_t adcMaxCode);

        /**
         * Convert a voltage (in mV) to DAC code
         */
        [[nodiscard]] etl::expected<uint16_t, Error> voltageToDACCode(float voltage);

        [[nodiscard]] etl::expected<uint32_t, Error> getMaxADCCode(ADC_HandleTypeDef& adcHandle);

        [[nodiscard]] etl::expected<float, Error> clampVoltage(float voltage, float lower, float upper);

        /**
         * @note The ADL5330 gain is equal to G (dB) = Pout - Pin, so ADL5330MinGain < Pout - Pin < ADL5330MaxGain
         *       For fixed Pin, the achievable Pout range is: ADL5330MinGain + Pin < Pout < ADL5330MaxGain + Pin
         *
         *       This function will return a VOLTAGE_OUT_OF_BOUNDS error if the given
         *       targetOutputPowerDBm results in a voltage outside the range [AD8318MinVset, AD8318MaxVset]
         */
        [[nodiscard]] etl::expected<float, Error> targetOutputPowerToVoltage(float targetOutputPowerDBm);

        [[nodiscard]] etl::expected<float, Error> targetGainToVoltage(float targetGainDB);

        [[nodiscard]] etl::expected<float, Error> voltageToGain(float gainVoltage);

        /**
         * @brief Calibrates gain ADC, vset DAC, and then powers up the rx frontend
         *
         * @param agcMode If set to true, agc mode is chosen. Otherwise, constant gain mode
         */
        [[nodiscard]] etl::expected<void, Error> enableUhfRxFrontend(float vset, bool agcMode);

        ADC_HandleTypeDef* ad8318TempADCHandle = nullptr;
        ADC_HandleTypeDef* gainADCHandle = nullptr;
        DAC_HandleTypeDef* vsetDACHandle = nullptr;
        uint32_t dacChannelNumber = DAC_CHANNEL_2;
        etl::delegate<float()> getVref;

        /// State of external frontend chains
        bool txUhfActive = false;
        bool sbandActive = false;
        UHFRxFrontendState rxFrontendState = UHFRxFrontendState::OFF;

        uint32_t ad8318TempADCMaxCode = (1U << 12) - 1;
        uint32_t gainADCMaxCode = (1U << 12) - 1;
        static constexpr uint32_t MaxDacCode = (1U << 12) - 1;

        /// Stores dac code that corresponds to the user targetOutputPowerDBm, given in enableUhfRxFrontendAutomaticGainControl()
        uint16_t ad8318VsetDacCode;

        static constexpr uint16_t TempConversionMaxDelayMs = 15;
        static constexpr uint16_t GainAGCConversionMaxDelayMs = 15;
        /// Turn-on delay for the frontends (defined mainly by the FPF2701 load switch, which needs 10.2 ms to open)
        static constexpr uint16_t FrontendTurnOnDelayMs = 15;

        /// ADL5330 (Rev. B), Table 1, 450 MHz
        static constexpr float ADL5330MinVgain = 100.0F; // mV
        static constexpr float ADL5330MaxVgain = 1400.0F; // mV
        static constexpr float ADL5330MinGain = -35.0F; // dB
        static constexpr float ADL5330MaxGain = 22.0F; // dB
        static constexpr float ADL5330VgainSlope = (ADL5330MaxVgain - ADL5330MinVgain) / (ADL5330MaxGain - ADL5330MinGain); // mV/dB

        /// Approximate boundaries for the ADL5330 & AD8318 application circuit
        /// ADL5330 (Rev. B), Figure 38
        static constexpr float AD8318MinVset = 500.0F; // mV
        static constexpr float AD8318MaxVset = 2100.0F; // mV

        /// AD8318 AD, Temperature Sensor chapter
        static constexpr float AD8318TempSensorSlope = 2.0F;  // mv/degree
        static constexpr float AD8318TempSensorVref = 600.0F; // mv
        static constexpr float AD8318TempSensorTref = 27.0F;  // degrees

        /// AD8318 AD, Controller Mode chapter (two point calibration)
        float ad8318Vset1 = 0; // mV
        float ad8318Vset2 = 0; // mV
        float ad8318Pout1 = 0; // dBm
        float ad8318Pout2 = 0; // dBm
        float ad8318VsetSlope = 0; // mV/dBm

        static constexpr float VoltageBoundTolerance = 30; // mV
    };

    extern ExternalFrontendUtilities externalFrontendUtils;
} // namespace ExternalFrontend
#pragma once

#include <cstdint>
#include "etl/optional.h"
#include "stm32h7xx_hal_gpio.h"

namespace ExternalFrontend {
    /**
     * A class that encapsulates all necessary functionality and state for the
     * external frontend of the SatNOGS-COMMS board.
     */
    class ExternalFrontendUtilities {
    public:
        ExternalFrontendUtilities() = default;

        /**
         * @note All chains will be set to inactive by default.
         */
        void initializeResources(ADC_HandleTypeDef* hadc_Temp, ADC_HandleTypeDef* hadc_Gain,
                                  DAC_HandleTypeDef* hdac_SetpointVoltage);

        /// Flags that get set to true from a callback, in case the current limiters FLAGB pin goes
        ///  LOW, due to output overcurrent, input undervoltage or overheating.
        ///  TODO FDIR
        volatile bool ExternalFrontend_currentLimiterAlertTxUHF = false;
        volatile bool ExternalFrontend_currentLimiterAlertRxUHF = false;
        volatile bool ExternalFrontend_currentLimiterAlertSBAND = false;

        /**
         * @brief Activate 5V supply, LNA, AMPLIFIER and AGC  for UHF-Tx frontend.
         *
         * @warning The function has effect ONLY IF the frontend was previously disabled.
         *
         * @details The UHF-Rx frontend uses an LNA (TQP3M9036) to amplify the signal upon reception. There is a second
         *       stage amplification by the ADL5330, which can optionally be driven by an AGC (AD83180). Should the
        *        AGC be used, the envelope of the signal at the output of ADL5330 is constant. Look figure 45 of the
        *        ADL8318 datasheet for a visual explanation.
        *
        *  @note The default parameters are for no AGC and an approximately 0 dB gain at 450 MHz
        *
         * @param agcEnabled   Whether the AGC will be used to drive the amplifier.
         * @param userSetPointVoltage This parameter should be interpreted as follows:
         *                        agcEnabled == False: Sets a constant amplifier gain. Look TYPICAL PERFORMANCE CHARACTERISTICS
         *                        section of the ADL5330 datasheet.
         *                        agcEnabled == True: The AGC tries to match it's input voltage (a fraction of
         *                        the amplifier's output voltage) with setPointVoltage.
         *                        In any case, the maximum value  should be between 0 and MaxSetPointVoltage. An out of bounds value is clipped.
         *
         * @returns Whether the operation succeeded or not.
         */
        [[nodiscard]] bool enableUhfRxFrontend(bool agcEnabled = false, float userSetPointVoltage = 0.9);

        /**
         * @brief Activate 5V supply for UHF-Tx frontend
         */
        void enableUhfTxFrontend();

        /**
         * @brief Activate 5V supply for SBAND-Tx frontend
         */
        void enableSbandTxFrontend();

        void disableUhfRxFrontend();
        void disableUhfTxFrontend();
        void disableSbandTxFrontend();

        /**
         * @brief Read the temperature from the integrated analog sensor of the AGC peripheral.
         *        The reference voltage is 600mV in 27 degrees Celsius, and the slope is 2mV/degree
         * @returns The temperature in Celsius, if the Rx-UHF chain is active and the ADC conversion succeeded.
         */
        [[nodiscard]] etl::optional<float> readAGCTemperature();

        /**
         * @brief Freeze AGC to its current gain
         * @returns Whether operation was successful or not.
         */
        [[nodiscard]] bool freezeAGC();

        /**
         * @brief Return AGC to normal operation.
         * @returns Whether operation was successful or not. False will also be returned in case the AGC is switched out.
         */
        [[nodiscard]] bool releaseAGC() const;

        /**
         * @brief Debugging function.
         */
        [[nodiscard]] etl::optional<float> readGainVoltage();

    private:
        /// State of external frontend chains
        bool txUhfActive;
        bool rxUhfActive;
        bool sbandActive;

        ///  Determines whether automatic gain control mode is used for the RX_UHF frontend, or a constant
        ///  gain is applied instead.
        bool agcSwitchedIn;

        /// Stores user setPoint voltage (12 bit resolution)
        uint32_t userSetPointVoltage;

        /// Handlers for ADC,DAC conversions (via polling)
        ADC_HandleTypeDef* hadcTemp;            // ADC handle for reading temperature pin of AGC
        ADC_HandleTypeDef* hadcGain;            // ADC handle for reading the current gain of the agc
        DAC_HandleTypeDef* hdacSetpointVoltage; // DAC handle for writing the setpoint voltage

        /// Delays for waiting
        static constexpr uint16_t TempConversionMaxDelayMs = 15;
        static constexpr uint16_t GainAGCConversionMaxDelayMs = 15;

        static constexpr float MaxSetPointVoltage = 1.4F;

        static constexpr float ReferenceVoltage = 3.28;  // VDDA

        /// AGC Temperature linear characteristic variables
        static constexpr float Slope = 2.0F;  // mv/degree
        static constexpr float Vref = 600.0F; // mv
        static constexpr float Tref = 27.0F;  // degrees
    };

    extern ExternalFrontendUtilities externalFrontendUtils;
} // namespace ExternalFrontend
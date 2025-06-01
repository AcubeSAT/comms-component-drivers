#pragma once

#include <cstdint>
#include "etl/optional.h"
#include "stm32h7xx_hal_gpio.h"

/// Flags that get set to true from a callback, in case the current limiters FLAGB pin goes
///  LOW, due to output overcurrent, input undervoltage or overheating. The user may read them.
inline volatile bool ExternalFrontend_currentLimiterAlertTxUHF = false;
inline volatile bool ExternalFrontend_currentLimiterAlertRxUHF = false;
inline volatile bool ExternalFrontend_currentLimiterAlertSBAND = false;

namespace ExternalFrontend {
    enum class ExternalFrontendChain {
        TX_UHF,
        RX_UHF,
        TX_SBAND
    };

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
        void initializeResources(ADC_HandleTypeDef* hadc_Temp, ADC_HandleTypeDef* hadc_OutVoltAgc,
                                  DAC_HandleTypeDef* hdac_SetpointVoltage);

        [[nodiscard]] bool getAgcSwitchedIn() const {
            return agcSwitchedIn;
        }

        /**
         *  @brief Open current limiters and enable components.
         *  @param agcSwitchedIn Determines whether the AGC should be enabled or not (has effect only for RX_UHF chain).
         *  @note There is a turn on time determined mainly by the current limiters. The user should begin
         *        utilizing the frontend 16ms after calling this function.
         *
         *    TODO calculate and set here the setpoint voltage as a function of a more useful input parameter,
         *         like the desired PA output power in dBm
         */
        void enableExternalFrontend(ExternalFrontendChain chain, bool agcSwitchedIn = false);

        void disableExternalFrontend(ExternalFrontendChain chain);

        /**
         * @brief Read the temperature from the integrated analog sensor of the AGC peripheral.
         *        The reference voltage is 600mV in 27 degrees Celsius, and the slope is 2mV/degree
         * @returns The temperature in Celsius, if the Rx-UHF chain is active and the ADC conversion succeeded.
         */
        [[nodiscard]] etl::optional<float> readAGCTemperature();

        /**
         * @brief Freeze AGC to its current gain
         * @returns Whether operation was successful or not. False will also be returned in case the AGC is switched out.
         *          In case of a successful operation, the ADC output is returned as well (for diagnostic purposes).
         */
        etl::pair<bool, uint16_t> freezeAGC();

        /**
         * @brief Return AGC to normal operation.
         * @returns Whether operation was successful or not. False will also be returned in case the AGC is switched out.
         */
        [[nodiscard]] bool unfreezeAGC() const;

    private:
        /// State of external frontend chains
        bool txUhfActive;
        bool rxUhfActive;
        bool sbandActive;

        ///  Determines whether automatic gain control mode is used for the RX_UHF frontend, or a constant
        ///  gain is applied instead.
        bool agcSwitchedIn;

        uint32_t setPointVoltage;

        /// Handlers for ADC,DAC conversions (via polling)
        ADC_HandleTypeDef* hadcTemp;            // ADC handle for reading temperature pin of AGC
        ADC_HandleTypeDef* hadcOutVoltAgc;      // ADC handle for reading the AGC's current out voltage (which is used to set the AMP gain, if the AGC in switched in)
        DAC_HandleTypeDef* hdacSetpointVoltage; // DAC handle for writing the setpoint voltage

        /// Converted voltages are stored here
        uint32_t voltageBufferTemp;
        uint32_t voltageBufferOutVoltAGC;

        /// Delays for waiting    // TODO find proper timings
        static constexpr uint16_t tempConversionMaxDelayMs = 15;
        static constexpr uint16_t outVoltAGCConversionMaxDelayMs = 15;

        /// AGC Temperature linear characteristic variables
        static constexpr float Slope = 2.0F;  // mv/degree
        static constexpr float Vref = 600.0F; // mv
        static constexpr float Tref = 27.0F;  // degrees

        /**
         * @brief Read the current output voltage of the AGC ()
         * @note Used by freezeAGC()
         * @returns Whether the operation was successful or not
         */
        bool readOutVoltageAGC();
    };

    extern ExternalFrontendUtilities externalFrontendUtils;
} // namespace ExternalFrontend
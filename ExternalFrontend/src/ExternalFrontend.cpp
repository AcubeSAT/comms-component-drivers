#include "main.h"
#include "ExternalFrontend.hpp"

#include <MCUTemperatureTask.hpp>

#include "Task.hpp"
namespace ExternalFrontend {
    void ExternalFrontendUtilities::enableExternalFrontend(const ExternalFrontendChain chain, bool agcSwitchedIn) {
        switch (chain) {
            case ExternalFrontendChain::TX_UHF:
                if (!txUhfActive) {
                    // enable current limiter (active low logic)
                    HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_RESET);
                    txUhfActive = true;
                }
                break;
            case ExternalFrontendChain::RX_UHF:
                if (!rxUhfActive) {
                    // enable current limiter (active low logic)
                    HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_RESET);
                    // enable the AMPLIFIER, LNA and the AGC (active high logic)
                    HAL_GPIO_WritePin(EN_UHF_AMP_RX__GPIO_Port, EN_UHF_AMP_RX__Pin, GPIO_PIN_SET);
                    rxUhfActive = true;

                    // switch in the AGC (if requested)
                    if (agcSwitchedIn) {
                        HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_RESET);
                    } else {
                        HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_SET);
                    }
                }
                break;
            case ExternalFrontendChain::TX_SBAND:
                if (!sbandActive) {
                    // enable current limiter (active low logic)
                    HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_RESET);
                    sbandActive = true;
                }
                break;
        }
    }

    void ExternalFrontendUtilities::disableExternalFrontend(ExternalFrontendChain chain) {
        switch (chain) {
            case ExternalFrontendChain::TX_UHF:
                if (txUhfActive) {
                    // disable current limiter (active low logic)
                    HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_SET);
                    txUhfActive = false;
                }
                break;
            case ExternalFrontendChain::RX_UHF:
                if (rxUhfActive) {
                    // disable current limiter (active low logic)
                    HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_SET);
                    // disable the AMPLIFIER, LNA and the AGC (active high logic)
                    HAL_GPIO_WritePin(EN_UHF_AMP_RX__GPIO_Port, EN_UHF_AMP_RX__Pin, GPIO_PIN_RESET);
                    rxUhfActive = false;
                }
                break;
            case ExternalFrontendChain::TX_SBAND:
                if (sbandActive) {
                    // disable current limiter (active low logic)
                    HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_SET);
                    sbandActive = false;
                }
                break;
        }
    }

    etl::optional<float> ExternalFrontendUtilities::readAGCTemperature() {
        if (!rxUhfActive) {
            return {};
        }

        // Calibrate peripheral for better accuracy
        if (HAL_ADCEx_Calibration_Start(hadcTemp, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
            return {};
        }

        if (HAL_ADC_PollForConversion(hadcTemp, tempConversionMaxDelayMs) != HAL_OK) {
            return {};
        }

        voltageBufferTemp = HAL_ADC_GetValue(hadcTemp);
        HAL_ADC_Stop(hadcTemp);

        // TODO confirm the 3.3 voltage
        const float voltageFloat = static_cast<float>(voltageBufferTemp) * (3.3F/ 65536.0F);  // assuming 3.3V power supply and 16 bit resolution
        return (1000.0F * voltageFloat - Vref) / Slope + Tref;
    }

    bool ExternalFrontendUtilities:: readOutVoltageAGC() {
        // Calibrate peripheral for better accuracy
        if (HAL_ADCEx_Calibration_Start(hadcOutVoltAgc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
            return {};
        }

        if (HAL_ADC_PollForConversion(hadcOutVoltAgc, outVoltAGCConversionMaxDelayMs) != HAL_OK) {
            return {};
        }

        voltageBufferOutVoltAGC = HAL_ADC_GetValue(hadcOutVoltAgc);
        HAL_ADC_Stop(hadcOutVoltAgc);
        return true;
    }

    etl::pair<bool, uint16_t> ExternalFrontendUtilities::freezeAGC() {
        if (!rxUhfActive || !agcSwitchedIn) {
            return etl::make_pair(false, 0);
        }

        // read current output value by agc
        if (!readOutVoltageAGC()) {
            return etl::make_pair(false, 0);
        }

        // use it as the new setpoint voltage
        if (HAL_DAC_Start(hdacSetpointVoltage, DAC_CHANNEL_2) != HAL_OK) {
            return etl::make_pair(false, static_cast<uint16_t>(voltageBufferOutVoltAGC));
        }

        if (HAL_DAC_SetValue(hdacSetpointVoltage, DAC_CHANNEL_2, DAC_ALIGN_12B_R, voltageBufferOutVoltAGC) != HAL_OK) {
            return etl::make_pair(false, static_cast<uint16_t>(voltageBufferOutVoltAGC));
        }

        // switch out the AGC
        HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_SET);
        return etl::make_pair(true, static_cast<uint16_t>(voltageBufferOutVoltAGC));
    }

    bool ExternalFrontendUtilities::unfreezeAGC() const {
        if (!rxUhfActive || !agcSwitchedIn) {
            return false;
        }

        // use the defined setpoint voltage
        if (HAL_DAC_Start(hdacSetpointVoltage, DAC_CHANNEL_2) != HAL_OK) {
            return false;
        }

        if (HAL_DAC_SetValue(hdacSetpointVoltage, DAC_CHANNEL_2, DAC_ALIGN_12B_R, setPointVoltage) != HAL_OK) {
            return false;
        }

        // switch in the AGC
        HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_RESET);
        return true;
    }

    ExternalFrontendUtilities externalFrontendUtils = ExternalFrontendUtilities();
} // namespace ExternalFrontend
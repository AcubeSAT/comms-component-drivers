#include "main.h"
#include "ExternalFrontend.hpp"

#include <MCUTemperatureTask.hpp>

#include "Task.hpp"
namespace ExternalFrontend {
    // definition
    ExternalFrontendUtilities externalFrontendUtils = ExternalFrontend::ExternalFrontendUtilities();

    void ExternalFrontendUtilities::initializeResources(ADC_HandleTypeDef* hadc_Temp, ADC_HandleTypeDef* hadc_Gain,
                                  DAC_HandleTypeDef* hdac_SetpointVoltage) {
        txUhfActive = false;
        rxUhfActive = false;
        sbandActive = false;

        hadcTemp = hadc_Temp;
        hadcGain = hadc_Gain;
        hdacSetpointVoltage = hdac_SetpointVoltage;

        HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_SET);

        // ensure DAC and ADCs are closed
        HAL_DAC_Stop(hdacSetpointVoltage, DAC_CHANNEL_2);
        HAL_ADC_Stop(hadcGain);
        HAL_ADC_Stop(hadcTemp);
    }

    bool ExternalFrontendUtilities::enableUhfRxFrontend(bool agcEnabled, float setPointVlt) {
        if (!rxUhfActive) {
            // clip out of range values
            if (agcEnabled) {
                if (setPointVlt < MinSetPointVoltageAGCMode) {
                    setPointVlt = MinSetPointVoltageAGCMode;
                }

                if (setPointVlt > MaxSetPointVoltageAGCMode) {
                    setPointVlt = MaxSetPointVoltageAGCMode;
                }
            } else {
                if (setPointVlt < MinSetPointVoltageAmplifierMode) {
                    setPointVlt = MinSetPointVoltageAmplifierMode;
                }

                if (setPointVlt > MaxSetPointVoltageAmplifierMode) {
                    setPointVlt = MaxSetPointVoltageAmplifierMode;
                }
            }

            // Store the value that is written to the DAC for future usage. It is assumed that the resolution is 12bits
            userSetPointVoltage = (setPointVlt / ReferenceVoltage) * 4096.0F;

            // switch in the AGC (if requested)
            agcSwitchedIn = agcEnabled;
            if (agcEnabled) {
                HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_SET);
            }

            // Calibrate and enable setpoint voltage dac and gain adc here. The calibration is necessary to do each time,
            // so that the large temperature variations do not affect the results. The activation is also done here,
            // to avoid overhead in freezeAGC(), unfreezeAGC().
            if (HAL_ADCEx_Calibration_Start(hadcGain, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
                return false;
            }
            HAL_ADC_Start(hadcGain);

            DAC_ChannelConfTypeDef sConfig = {0}; // this struct is used by the function just to return the trimming value (we dont need it anywhere else)
            if (HAL_DACEx_SelfCalibrate(hdacSetpointVoltage, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
                return false;
            }
            HAL_DAC_Start(hdacSetpointVoltage, DAC_CHANNEL_2);
            HAL_DAC_SetValue(hdacSetpointVoltage, DAC_CHANNEL_2, DAC_ALIGN_12B_R,userSetPointVoltage);
            // ensure the value is set correctly
            if (HAL_DAC_GetValue(hdacSetpointVoltage, DAC_CHANNEL_2) != userSetPointVoltage) {
                return false;
            }

            // enable current limiter (active low logic)
            HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_RESET);
            // enable the AMPLIFIER, LNA and the AGC (active high logic)
            HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_SET);

            // wait for all components to activate
            vTaskDelay(pdMS_TO_TICKS(TurnOnDelayMs));
            rxUhfActive = true;
            return true;
        }
        return false;
    }

    void ExternalFrontendUtilities::enableUhfTxFrontend() {
        if (!txUhfActive) {
            // enable current limiter (active low logic)
            HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(TurnOnDelayMs));
            txUhfActive = true;
        }
    }

    void ExternalFrontendUtilities::enableSbandTxFrontend() {
        if (!sbandActive) {
            // enable current limiter (active low logic)
            HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(TurnOnDelayMs));
            sbandActive = true;
        }
    }

    void ExternalFrontendUtilities::disableUhfRxFrontend() {
        if (rxUhfActive) {
            // disable current limiter (active low logic)
            HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_SET);
            // disable the AMPLIFIER, LNA and the AGC (active high logic)
            HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_RESET);
            // disable dac and adc
            HAL_DAC_Stop(hdacSetpointVoltage, DAC_CHANNEL_2);
            HAL_ADC_Stop(hadcGain);
            rxUhfActive = false;
        }
    }

    void ExternalFrontendUtilities::disableUhfTxFrontend() {
        if (txUhfActive) {
            // disable current limiter (active low logic)
            HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_SET);
            txUhfActive = false;
        }
    }

    void ExternalFrontendUtilities::disableSbandTxFrontend() {
        if (sbandActive) {
            // disable current limiter (active low logic)
            HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_SET);
            sbandActive = false;
        }
    }

    etl::optional<float> ExternalFrontendUtilities::readAGCTemperature() {
        if (!rxUhfActive) {
            return {};
        }

        // calibrate gain adc for better accuracy
        if (HAL_ADCEx_Calibration_Start(hadcTemp, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
            return {};
        }

        if (HAL_ADC_Start(hadcTemp) != HAL_OK) {
            return {};
        }

        if (HAL_ADC_PollForConversion(hadcTemp, TempConversionMaxDelayMs) != HAL_OK) {
            return {};
        }

        const uint32_t adcVoltage = HAL_ADC_GetValue(hadcTemp);
        HAL_ADC_Stop(hadcTemp);

        const float voltageFloat = (static_cast<float>(adcVoltage) / 4096.0F) * ReferenceVoltage;
        return (1000.0F * voltageFloat - Vref) / Slope + Tref;
    }

    bool ExternalFrontendUtilities::freezeAGC() {
        if (!rxUhfActive || !agcSwitchedIn) {
            return false;
        }

        // read current output value by agc
        if (HAL_ADC_PollForConversion(hadcGain, GainAGCConversionMaxDelayMs) != HAL_OK) {
            return {};
        }

        const uint32_t adcVoltage = HAL_ADC_GetValue(hadcGain);
        // calculate downscaled voltage
        const float adcVoltageFloatDownscaled = (static_cast<float>(adcVoltage) / 4096.0F) * ReferenceVoltage * agcOutVoltageDownscaleFactor;
        const uint32_t adcVoltageDownscaled = (adcVoltageFloatDownscaled / ReferenceVoltage) * 4096.0F;

        // use it as the new setpoint voltage (
        HAL_DAC_SetValue(hdacSetpointVoltage, DAC_CHANNEL_2, DAC_ALIGN_12B_R,adcVoltageDownscaled);

        // ensure the value is set correctly
        if (HAL_DAC_GetValue(hdacSetpointVoltage, DAC_CHANNEL_2) != adcVoltage) {
            return false;
        }

        // switch out the AGC
        HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_SET);
        return true;
    }

    bool ExternalFrontendUtilities::releaseAGC() const {
        if (!rxUhfActive || !agcSwitchedIn) {
            return false;
        }

        // use the defined setpoint voltage
        HAL_DAC_SetValue(hdacSetpointVoltage, DAC_CHANNEL_2, DAC_ALIGN_12B_R,userSetPointVoltage);

        // ensure the value is set correctly
        if (HAL_DAC_GetValue(hdacSetpointVoltage, DAC_CHANNEL_2) != userSetPointVoltage) {
            return false;
        }

        // switch in the AGC
        HAL_GPIO_WritePin(EN_AGC_UHF_GPIO_Port, EN_AGC_UHF_Pin, GPIO_PIN_RESET);
        return true;
    }

    etl::optional<float> ExternalFrontendUtilities::readGainVoltage() {
        if (!rxUhfActive) {
            return {};
        }

        // read current output value by agc
        if (HAL_ADC_PollForConversion(hadcGain, GainAGCConversionMaxDelayMs) != HAL_OK) {
            return {};
        }

        const uint32_t adcVoltage = HAL_ADC_GetValue(hadcGain);

        return (static_cast<float>(adcVoltage) / 4096.0F) * ReferenceVoltage;
    }
} // namespace ExternalFrontend
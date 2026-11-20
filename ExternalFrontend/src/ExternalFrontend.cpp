#include <cmath>
#include "main.h"
#include "ExternalFrontend.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace ExternalFrontend {
    ExternalFrontendUtilities externalFrontendUtils = ExternalFrontendUtilities();

    etl::expected<float, Error> ExternalFrontendUtilities::adcCodeToVoltage(uint16_t adcCode, uint32_t adcMaxCode) {
        const float vref = getVref();
        if (vref <= 0) {
            return etl::unexpected{Error{ErrorType::INVALID_VREF}};
        }

        if (adcCode > adcMaxCode) {
            return etl::unexpected{Error{ErrorType::INVALID_CODE}};
        }

        return (static_cast<float>(adcCode) / static_cast<float>(adcMaxCode)) * vref;
    }

    etl::expected<uint16_t, Error> ExternalFrontendUtilities::voltageToDACCode(float voltage) {
        const float vref = getVref();
        if (vref <= 0) {
            return etl::unexpected{Error{ErrorType::INVALID_VREF}};
        }

        if (voltage < 0 || voltage > vref) {
            return etl::unexpected<Error>{{ErrorType::VOLTAGE_OUT_OF_BOUNDS}};
        }

        float dacCodeFloat = (voltage / vref) * MaxDacCode;
        return static_cast<uint16_t>(std::roundf(dacCodeFloat));
    }

    etl::expected<uint32_t, Error> ExternalFrontendUtilities::getMaxADCCode(ADC_HandleTypeDef& adcHandle) {
        switch (adcHandle.Init.Resolution) {
        case ADC_RESOLUTION_16B:
            return (1U << 16) - 1;
        case ADC_RESOLUTION_14B:
            return (1U << 14) - 1;
        case ADC_RESOLUTION_12B:
            return (1U << 12) - 1;
        case ADC_RESOLUTION_10B:
            return (1U << 10) - 1;
        case ADC_RESOLUTION_8B:
            return (1U << 8) - 1;
        default:
            return etl::unexpected<Error>{{ErrorType::INVALID_ADC_OR_DAC_RESOLUTION}};
        }
    }

    etl::expected<float, Error> ExternalFrontendUtilities::clampVoltage(float voltage, float lower, float upper) {
        if (voltage < lower - VoltageBoundTolerance) {
            return etl::unexpected<Error>{{ErrorType::VOLTAGE_OUT_OF_BOUNDS}};
        }
        if (voltage < lower) {
            return lower;
        }
        if (voltage <= upper) {
            return voltage;
        }
        if (voltage <= upper + VoltageBoundTolerance) {
            return upper;
        }

        return etl::unexpected<Error>{{ErrorType::VOLTAGE_OUT_OF_BOUNDS}};
    }

    etl::expected<float, Error> ExternalFrontendUtilities::targetOutputPowerToVoltage(float targetOutputPowerDBm) {
        const float vref = getVref();
        if (vref <= 0) {
            return etl::unexpected{Error{ErrorType::INVALID_VREF}};
        }

        const float vset = ad8318VsetSlope * (targetOutputPowerDBm - ad8318Pout1) + ad8318Vset1;
        if (vset < AD8318MinVset || vset > AD8318MaxVset) {
            return etl::unexpected<Error>{{ErrorType::VOLTAGE_OUT_OF_BOUNDS}};
        }
        return vset;
    }

    etl::expected<float, Error> ExternalFrontendUtilities::targetGainToVoltage(float targetGainDB) {
        if (targetGainDB < ADL5330MinGain || targetGainDB > ADL5330MaxGain) {
            return etl::unexpected<Error>{{ErrorType::GAIN_OUT_OF_BOUNDS}};
        }

        return ADL5330VgainSlope * (targetGainDB - ADL5330MinGain) + ADL5330MinVgain;
    }

    etl::expected<float, Error> ExternalFrontendUtilities::voltageToGain(float gainVoltage) {
        auto clampedVoltageStatus = clampVoltage(gainVoltage, ADL5330MinVgain, ADL5330MaxVgain);
        if (!clampedVoltageStatus.has_value()) {
            return clampedVoltageStatus;
        }

        return (1 / ADL5330VgainSlope) * (clampedVoltageStatus.value() - ADL5330MinVgain) + ADL5330MinGain;
    }

    etl::expected<void, Error> ExternalFrontendUtilities::initialize(
            ADC_HandleTypeDef* ad8318Temp_ADCHandle,
            ADC_HandleTypeDef* gain_ADCHandle,
            DAC_HandleTypeDef* vset_DACHandle,
            uint32_t dac_ChannelNumber,
            etl::delegate<float()> get_Vref,
            float ad8318_Vset1,
            float ad8318_Vset2,
            float ad8318_Pout1,
            float ad8318_Pout2) {

        if (ad8318Temp_ADCHandle == nullptr || gain_ADCHandle == nullptr || vset_DACHandle == nullptr) {
            return etl::unexpected{Error{ErrorType::NULL_PTR}};
        }

        if (!get_Vref.is_valid()) {
            return etl::unexpected{Error{ErrorType::INVALID_DELEGATE}};
        }

        ad8318TempADCHandle = ad8318Temp_ADCHandle;
        gainADCHandle = gain_ADCHandle;
        vsetDACHandle = vset_DACHandle;
        dacChannelNumber = dac_ChannelNumber;
        getVref = get_Vref;

        if ((ad8318_Vset1 == ad8318_Vset2) || (ad8318_Pout1 == ad8318_Pout2)) {
            return etl::unexpected<Error>{{ErrorType::IDENTICAL_CALIBRATION_POINTS}};
        }
        ad8318Vset1 = ad8318_Vset1;
        ad8318Vset2 = ad8318_Vset2;
        ad8318Pout1 = ad8318_Pout1;
        ad8318Pout2 = ad8318_Pout2;
        ad8318VsetSlope = (ad8318Vset2 - ad8318Vset1) / (ad8318Pout2 - ad8318Pout1);

        if (auto status = getMaxADCCode(*ad8318TempADCHandle); !status.has_value()) {
            return etl::unexpected{Error{status.error()}};
        } else {
            ad8318TempADCMaxCode = status.value();
        }

        if (auto status = getMaxADCCode(*gainADCHandle); !status.has_value()) {
            return etl::unexpected{Error{status.error()}};
        } else {
            gainADCMaxCode = status.value();
        }

        if (dacChannelNumber != DAC_CHANNEL_1 && dacChannelNumber != DAC_CHANNEL_2) {
            return etl::unexpected<Error>{{ErrorType::INVALID_DAC_CHANNEL_NUMBER}};
        }

        disableUhfTxFrontend();
        disableSbandTxFrontend();

        if (auto status = disableUhfRxFrontend(); !status.has_value()) {
            return status;
        }
        return {};
    }

    void ExternalFrontendUtilities::enableUhfTxFrontend() {
        if (!txUhfActive) {
            HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(FrontendTurnOnDelayMs));
            txUhfActive = true;
        }
    }

    void ExternalFrontendUtilities::enableSbandTxFrontend() {
        if (!sbandActive) {
            HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(FrontendTurnOnDelayMs));
            sbandActive = true;
        }
    }

    etl::expected<void, Error> ExternalFrontendUtilities::enableUhfRxFrontend(float vset, bool agcMode) {
        if (rxFrontendState != UHFRxFrontendState::OFF) {
            if (auto disableUhfRxFrontendStatus = disableUhfRxFrontend(); !disableUhfRxFrontendStatus.has_value()) {
                return disableUhfRxFrontendStatus;
            }
        }

        auto voltageToDacCodeStatus = voltageToDACCode(vset);
        if (!voltageToDacCodeStatus.has_value()) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{voltageToDacCodeStatus.error()};
        }

        if (auto status = HAL_ADCEx_Calibration_Start(gainADCHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
              status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::GAIN_ADC_FAILURE, status}};
        }

        if (auto status = HAL_ADC_Start(gainADCHandle); status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::GAIN_ADC_FAILURE, status}};
        }

        // This struct is used by the function just to return the trimming value (not needed anywhere else)
        DAC_ChannelConfTypeDef sConfig = {};
        if (auto status = HAL_DACEx_SelfCalibrate(vsetDACHandle, &sConfig, dacChannelNumber);
            status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        if (auto status = HAL_DAC_Start(vsetDACHandle, dacChannelNumber); status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        if (auto status = HAL_DAC_SetValue(vsetDACHandle, dacChannelNumber, DAC_ALIGN_12B_R,voltageToDacCodeStatus.value());
            status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        if (agcMode) {
            switchToAGCMode();
            ad8318VsetDacCode = voltageToDacCodeStatus.value();
            rxFrontendState = UHFRxFrontendState::AGC_MODE_UNLOCKED;
        } else {
            switchToConstantGainMode();
            rxFrontendState = UHFRxFrontendState::CONSTANT_GAIN_MODE;
        }

        HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(FrontendTurnOnDelayMs));
        return {};
    }

    etl::expected<void, Error> ExternalFrontendUtilities::enableUhfRxFrontendConstantGain(float targetGainDB) {
        auto targetGainVoltageStatus = targetGainToVoltage(targetGainDB);
        if (!targetGainVoltageStatus.has_value()) {
            return etl::unexpected{targetGainVoltageStatus.error()};
        }

        if (auto enableUhfRxFrontendStatus = enableUhfRxFrontend(targetGainVoltageStatus.value(), false);
            !enableUhfRxFrontendStatus.has_value()) {
            return etl::unexpected{enableUhfRxFrontendStatus.error()};
        }
        return {};
    }

    etl::expected<void, Error> ExternalFrontendUtilities::enableUhfRxFrontendAutomaticGainControl(float targetOutputPowerDBm) {
        auto targetOutputPowerToVoltageStatus = targetOutputPowerToVoltage(targetOutputPowerDBm);
        if (!targetOutputPowerToVoltageStatus.has_value()) {
            return etl::unexpected{targetOutputPowerToVoltageStatus.error()};
        }

        if (auto enableUhfRxFrontendStatus = enableUhfRxFrontend(targetOutputPowerToVoltageStatus.value(), true);
            !enableUhfRxFrontendStatus.has_value()) {
            return etl::unexpected{enableUhfRxFrontendStatus.error()};
        }
        return {};
    }

    etl::expected<void, Error> ExternalFrontendUtilities::disableUhfRxFrontend() {
        HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_RESET);

        etl::optional<Error> lastError;
        if (HAL_StatusTypeDef status = HAL_DAC_Stop(vsetDACHandle, dacChannelNumber); status != HAL_OK) {
            lastError = Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_Stop(gainADCHandle); status != HAL_OK) {
            lastError = Error{ErrorType::GAIN_ADC_FAILURE, status};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_Stop(ad8318TempADCHandle); status != HAL_OK) {
            lastError = Error{ErrorType::AD8318_TEMP_ADC_FAILURE, status};
        }

        rxFrontendState = UHFRxFrontendState::OFF;

        if (lastError.has_value()) {
            return etl::unexpected{lastError.value()};
        }
        return {};
    }

    void ExternalFrontendUtilities::disableUhfTxFrontend() {
        HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_SET);
        txUhfActive = false;
    }

    void ExternalFrontendUtilities::disableSbandTxFrontend() {
        HAL_GPIO_WritePin(EN_S_BAND_TX_GPIO_Port, EN_S_BAND_TX_Pin, GPIO_PIN_SET);
        sbandActive = false;
    }

    etl::expected<float, Error> ExternalFrontendUtilities::readAGCTemperature() {
        if (rxFrontendState == UHFRxFrontendState::OFF) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        if (HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(ad8318TempADCHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::AD8318_TEMP_ADC_FAILURE, status}};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_Start(ad8318TempADCHandle); status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::AD8318_TEMP_ADC_FAILURE, status}};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_PollForConversion(ad8318TempADCHandle, TempConversionMaxDelayMs);
            status != HAL_OK) {
            HAL_ADC_Stop(ad8318TempADCHandle);
            return etl::unexpected<Error>{{ErrorType::AD8318_TEMP_ADC_FAILURE, status}};
        }
        const uint32_t adcCode = HAL_ADC_GetValue(ad8318TempADCHandle);

        if (HAL_StatusTypeDef status = HAL_ADC_Stop(ad8318TempADCHandle); status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::AD8318_TEMP_ADC_FAILURE, status}};
        }

        auto temperatureVoltageStatus = adcCodeToVoltage(
            adcCode,
            ad8318TempADCMaxCode);
        if (!temperatureVoltageStatus.has_value()) {
            return temperatureVoltageStatus;
        }
        return (temperatureVoltageStatus.value() - AD8318TempSensorVref) / AD8318TempSensorSlope + AD8318TempSensorTref;
    }

    etl::expected<void, Error> ExternalFrontendUtilities::lockAGC() {
        if (rxFrontendState == UHFRxFrontendState::OFF) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        if (rxFrontendState != UHFRxFrontendState::AGC_MODE_UNLOCKED) {
            return etl::unexpected<Error>{{ErrorType::NOT_IN_UNLOCKED_AGC_MODE}};
        }

        // ADC is already active and calibrated from enableUhfRxFrontend(). A new calibration is avoided to
        // perform the lock faster
        if (auto status = HAL_ADC_PollForConversion(gainADCHandle, GainAGCConversionMaxDelayMs);
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::GAIN_ADC_FAILURE, status}};
        }
        const uint32_t adcCode = HAL_ADC_GetValue(gainADCHandle);

        auto adcCodeToVoltageStatus = adcCodeToVoltage(adcCode, gainADCMaxCode);
        if (!adcCodeToVoltageStatus.has_value()) {
            return etl::unexpected{adcCodeToVoltageStatus.error()};
        }

        auto clampVoltageStatus = clampVoltage(adcCodeToVoltageStatus.value(), ADL5330MinVgain, ADL5330MaxVgain);
        if (!clampVoltageStatus.has_value()) {
            return etl::unexpected{clampVoltageStatus.error()};
        }

        auto voltageToDacCodeStatus = voltageToDACCode(clampVoltageStatus.value());
        if (!voltageToDacCodeStatus.has_value()) {
            return etl::unexpected{voltageToDacCodeStatus.error()};
        }

        if (HAL_StatusTypeDef status = HAL_DAC_SetValue(vsetDACHandle, dacChannelNumber, DAC_ALIGN_12B_R,
            voltageToDacCodeStatus.value());
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        switchToConstantGainMode();
        rxFrontendState = UHFRxFrontendState::AGC_MODE_LOCKED;
        return {};
    }

    etl::expected<void, Error> ExternalFrontendUtilities::unlockAGC() {
        if (rxFrontendState == UHFRxFrontendState::OFF) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        if (rxFrontendState != UHFRxFrontendState::AGC_MODE_LOCKED) {
            return etl::unexpected<Error>{{ErrorType::NOT_IN_LOCKED_AGC_MODE}};
        }

        switchToAGCMode();
        if (HAL_StatusTypeDef status = HAL_DAC_SetValue(vsetDACHandle, dacChannelNumber, DAC_ALIGN_12B_R,ad8318VsetDacCode);
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        rxFrontendState = UHFRxFrontendState::AGC_MODE_UNLOCKED;
        return {};
    }

    etl::expected<float, Error> ExternalFrontendUtilities::readCurrentGain() {
        if (rxFrontendState == UHFRxFrontendState::OFF) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_PollForConversion(gainADCHandle, GainAGCConversionMaxDelayMs); status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::GAIN_ADC_FAILURE, status}};
        }

        auto gainVoltageStatus = adcCodeToVoltage(HAL_ADC_GetValue(gainADCHandle), gainADCMaxCode);
        if (!gainVoltageStatus.has_value()) {
            return gainVoltageStatus;
        }
        return voltageToGain(gainVoltageStatus.value());
    }
} // namespace ExternalFrontend
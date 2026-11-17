#include <cmath>
#include "main.h"
#include "ExternalFrontend.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace ExternalFrontend {
    ExternalFrontendUtilities externalFrontendUtils = ExternalFrontendUtilities();

    etl::expected<float, Error> ExternalFrontendUtilities::adcCodeToVoltage(uint16_t adcCode, uint16_t vrefIntCode, uint32_t adcMaxCode) {
        if (vrefIntCode == 0 || vrefIntCode > tempAndVrefIntADCMaxCode || adcCode > adcMaxCode) {
            return etl::unexpected{Error{ErrorType::INVALID_CODE}};
        }

        // VREFINT_CAL is calculated on 16 bit resolution on STM32H7. Scale to current resolution of ADC
        const float vrefIntCal = static_cast<float>(*VREFINT_CAL_ADDR)
                                 * static_cast<float>(tempAndVrefIntADCMaxCode) / ((1 << 16) - 1);

        const float vdda = VREFINT_CAL_VREF * (vrefIntCal / static_cast<float>(vrefIntCode));

        return (static_cast<float>(adcCode) / static_cast<float>(adcMaxCode)) * vdda;
    }

    etl::expected<uint16_t, Error> ExternalFrontendUtilities::voltageToDACCode(float voltage, uint16_t vrefIntCode) {
        if (vrefIntCode == 0) {
            return etl::unexpected{Error{ErrorType::INVALID_CODE}};
        }

        // VREFINT_CAL is calculated on 16 bit resolution on STM32H7. Scale to current resolution of ADC
        const float vrefIntCal = static_cast<float>(*VREFINT_CAL_ADDR)
                                 * static_cast<float>(tempAndVrefIntADCMaxCode) / ((1 << 16) - 1);

        float vdda = VREFINT_CAL_VREF * (vrefIntCal / static_cast<float>(vrefIntCode));

        if (voltage < 0 || voltage > vdda) {
            return etl::unexpected<Error>{{ErrorType::INVALID_VOLTAGE}};
        }

        float dacCodeFloat = (voltage / vdda) * MaxDacCode;
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

    etl::expected<float, Error> ExternalFrontendUtilities::targetOutputPowerToVoltage(float targetOutputPowerDBm) {
        const float vset = ad8318VsetSlope * (targetOutputPowerDBm - ad8318Pout1) + ad8318Vset1;

        // obtain upper boundary for vset
        auto status = getAgcTempAndVrefIntCodes();
        if (!status.has_value()) {
            return etl::unexpected{status.error()};
        }
        if (status.value().first == 0) {
            return etl::unexpected{Error{ErrorType::INVALID_CODE}};
        }
        //   VREFINT_CAL is calculated on 16 bit resolution on STM32H7. Scale to current resolution of ADC
        const float vrefIntCal = static_cast<float>(*VREFINT_CAL_ADDR)
                                 * static_cast<float>(tempAndVrefIntADCMaxCode) / ((1 << 16) - 1);
        float vdda = VREFINT_CAL_VREF * (vrefIntCal / static_cast<float>(status.value().first));

        if (vset < 0 || vset > vdda) {
            return etl::unexpected<Error>{{ErrorType::INVALID_VOLTAGE}};
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
        if (gainVoltage < ADL5330MinVgain || gainVoltage > ADL5330MaxVgain) {
            return etl::unexpected<Error>{{ErrorType::GAIN_OUT_OF_BOUNDS}};
        }

        return (1 / ADL5330VgainSlope) * (gainVoltage - ADL5330MinVgain) + ADL5330MinGain;
    }

    etl::expected<void, Error> ExternalFrontendUtilities::initialize(
            float ad8318_Vset1,
            float ad8318_Vset2,
            float ad8318_Pout1,
            float ad8318_Pout2) {

        if ((ad8318_Vset1 == ad8318_Vset2) || (ad8318_Pout1 == ad8318_Pout2)) {
            return etl::unexpected<Error>{{ErrorType::IDENTICAL_CALIBRATION_POINTS}};
        }
        ad8318Vset1 = ad8318_Vset1;
        ad8318Vset2 = ad8318_Vset2;
        ad8318Pout1 = ad8318_Pout1;
        ad8318Pout2 = ad8318_Pout2;
        ad8318VsetSlope = (ad8318Vset2 - ad8318Vset1) / (ad8318Pout2 - ad8318Pout1);

        if (auto status = getMaxADCCode(tempAndVrefIntADCHandle); !status.has_value()) {
            return etl::unexpected{Error{status.error()}};
        } else {
            tempAndVrefIntADCMaxCode = status.value();
        }

        if (auto status = getMaxADCCode(gainADCHandle); !status.has_value()) {
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
        if (rxUhfActive) {
            if (auto disableUhfRxFrontendStatus = disableUhfRxFrontend(); !disableUhfRxFrontendStatus.has_value()) {
                return disableUhfRxFrontendStatus;
            }
        }

        auto agcTempAndVrefIntStatus = getAgcTempAndVrefIntCodes();
        if (!agcTempAndVrefIntStatus.has_value()) {
            return etl::unexpected{agcTempAndVrefIntStatus.error()};
        }

        auto voltageToDacCodeStatus = voltageToDACCode(vset, agcTempAndVrefIntStatus.value().first);
        if (!voltageToDacCodeStatus.has_value()) {
            return etl::unexpected{voltageToDacCodeStatus.error()};
        }

        if (auto status = HAL_ADCEx_Calibration_Start(&gainADCHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
              status != HAL_OK) {
            return etl::unexpected{Error{ErrorType::GAIN_ADC_FAILURE, status}};
        }

        if (auto status = HAL_ADC_Start(&gainADCHandle); status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::GAIN_ADC_FAILURE, status}};
        }

        // This struct is used by the function just to return the trimming value (not needed anywhere else)
        DAC_ChannelConfTypeDef sConfig = {};
        if (auto status = HAL_DACEx_SelfCalibrate(&vsetDACHandle, &sConfig, dacChannelNumber);
            status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        if (auto status = HAL_DAC_Start(&vsetDACHandle, dacChannelNumber); status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        if (auto status = HAL_DAC_SetValue(&vsetDACHandle, dacChannelNumber, DAC_ALIGN_12B_R,voltageToDacCodeStatus.value());
            status != HAL_OK) {
            static_cast<void>(disableUhfRxFrontend());
            return etl::unexpected{Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        if (agcMode) {
            switchToAGCMode();
            adl5330VsetDacCode = voltageToDacCodeStatus.value();
        } else {
            switchToConstantGainMode();
        }

        HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(FrontendTurnOnDelayMs));
        rxUhfActive = true;
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
        if (HAL_StatusTypeDef status = HAL_DAC_Stop(&vsetDACHandle, dacChannelNumber); status != HAL_OK) {
            lastError = Error{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_Stop(&gainADCHandle); status != HAL_OK) {
            lastError = Error{ErrorType::GAIN_ADC_FAILURE, status};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_Stop(&tempAndVrefIntADCHandle); status != HAL_OK) {
            lastError = Error{ErrorType::TEMP_VREFINT_ADC_FAILURE, status};
        }

        adl5330VsetDacCode.reset();
        rxUhfActive = false;

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

    etl::expected<etl::pair<uint16_t, uint16_t>, Error> ExternalFrontendUtilities::getAgcTempAndVrefIntCodes() {
        if (HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(&tempAndVrefIntADCHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::TEMP_VREFINT_ADC_FAILURE, status}};
        }

        if (HAL_StatusTypeDef status = HAL_ADC_Start(&tempAndVrefIntADCHandle); status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::TEMP_VREFINT_ADC_FAILURE, status}};
        }

        // Obtain vrefint code
        if (HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&tempAndVrefIntADCHandle, TempConversionMaxDelayMs);
            status != HAL_OK) {
            HAL_ADC_Stop(&tempAndVrefIntADCHandle);
            return etl::unexpected<Error>{{ErrorType::TEMP_VREFINT_ADC_FAILURE, status}};
        }
        const uint32_t vrefintAdcCode = HAL_ADC_GetValue(&tempAndVrefIntADCHandle);

        // Obtain temperature code
        if (HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&tempAndVrefIntADCHandle, TempConversionMaxDelayMs);
            status != HAL_OK) {
            HAL_ADC_Stop(&tempAndVrefIntADCHandle);
            return etl::unexpected<Error>{{ErrorType::TEMP_VREFINT_ADC_FAILURE, status}};
        }
        const uint32_t adcVoltage = HAL_ADC_GetValue(&tempAndVrefIntADCHandle);

        if (HAL_StatusTypeDef status = HAL_ADC_Stop(&tempAndVrefIntADCHandle); status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::TEMP_VREFINT_ADC_FAILURE, status}};
        }

        return etl::make_pair(static_cast<uint16_t>(vrefintAdcCode), static_cast<uint16_t>(adcVoltage));
    }

    etl::expected<float, Error> ExternalFrontendUtilities::readAGCTemperature() {
        if (!rxUhfActive) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        auto status = getAgcTempAndVrefIntCodes();
        if (!status.has_value()) {
            return etl::unexpected{status.error()};
        }

        auto temperatureVoltageStatus = adcCodeToVoltage(
            status.value().second,
            status.value().first,
            tempAndVrefIntADCMaxCode);
        if (!temperatureVoltageStatus.has_value()) {
            return temperatureVoltageStatus;
        }
        return (temperatureVoltageStatus.value() - AD8318TempSensorVref) / AD8318TempSensorSlope + AD8318TempSensorTref;
    }

    etl::expected<void, Error> ExternalFrontendUtilities::lockAGC() {
        if (!rxUhfActive) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        if (!adl5330VsetDacCode.has_value()) {
            return etl::unexpected<Error>{{ErrorType::NOT_IN_AUTOMATIC_GAIN_CONTROL_MODE}};
        }

        // ADC is already active and calibrated from enableUhfRxFrontend(). A new calibration is avoided to
        // perform the lock faster
        if (auto status = HAL_ADC_PollForConversion(&gainADCHandle, GainAGCConversionMaxDelayMs);
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::GAIN_ADC_FAILURE, status}};
        }
        const uint32_t adcCode = HAL_ADC_GetValue(&gainADCHandle);

        if (HAL_StatusTypeDef status = HAL_DAC_SetValue(&vsetDACHandle, dacChannelNumber, DAC_ALIGN_12B_R,
            (adcCode * MaxDacCode) / gainADCMaxCode);
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        switchToConstantGainMode();
        return {};
    }

    etl::expected<void, Error> ExternalFrontendUtilities::unlockAGC() {
        if (!rxUhfActive) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        if (!adl5330VsetDacCode.has_value()) {
            return etl::unexpected<Error>{{ErrorType::NOT_IN_AUTOMATIC_GAIN_CONTROL_MODE}};
        }

        if (HAL_StatusTypeDef status = HAL_DAC_SetValue(&vsetDACHandle, dacChannelNumber, DAC_ALIGN_12B_R,adl5330VsetDacCode.value());
            status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::SETPOINT_VOLTAGE_DAC_FAILURE, status}};
        }

        switchToAGCMode();
        return {};
    }

    etl::expected<float, Error> ExternalFrontendUtilities::readCurrentGain() {
        if (!rxUhfActive) {
            return etl::unexpected<Error>{{ErrorType::UHF_RX_FRONT_END_DISABLED}};
        }

        // Obtain vrefInt code
        auto status = getAgcTempAndVrefIntCodes();
        if (!status.has_value()) {
            return etl::unexpected{status.error()};
        }
        uint16_t vrefIntCode = status.value().first;

        // Obtain current gain
        if (HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&gainADCHandle, GainAGCConversionMaxDelayMs); status != HAL_OK) {
            return etl::unexpected<Error>{{ErrorType::GAIN_ADC_FAILURE, status}};
        }

        auto gainVoltageStatus = adcCodeToVoltage(HAL_ADC_GetValue(&gainADCHandle), vrefIntCode, gainADCMaxCode);
        if (!gainVoltageStatus.has_value()) {
            return gainVoltageStatus;
        }
        return voltageToGain(gainVoltageStatus.value());
    }
} // namespace ExternalFrontend
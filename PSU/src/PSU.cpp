#include "PSU.hpp"
#include "stm32h7xx_hal_gpio.h"



namespace PSU {
    void PSU::enable_FPGA_PSU() {
        HAL_GPIO_WritePin(P5V_FPGA_EN_GPIO_Port, P5V_FPGA_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }
    void PSU::disable_FPGA_PSU(){
        HAL_GPIO_WritePin(P5V_FPGA_EN_GPIO_Port, P5V_FPGA_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    void PSU::enable_RF_PSU() {
        HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }
    void PSU::disable_RF_PSU() {
        HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    bool PSU::isOff(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
        GPIO_PinState pinstate_temporary;
        pinstate_temporary = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

        if (pinstate_temporary == GPIO_PIN_RESET)
            return true;
        else // is SET
            return false;
    }
    bool PSU::PG_read() {
        if ( isOff(P5V_FPGA_PG_GPIO_Port, P5V_FPGA_PG_Pin)
        || isOff(P5V_RF_PG_GPIO_Port, P5V_RF_PG_Pin)
        || isOff(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin) )
            return false;
        else
            return true;
    }

    void PSU::solve_PG_fault() {

        // P5V_FPGA_PG error fault detection
        while(isOff(P5V_FPGA_PG_GPIO_Port, P5V_FPGA_PG_Pin)){
            disable_FPGA_PSU();
            enable_FPGA_PSU();
        }

        // (P5V) RF_PG error fault detection
        while(isOff(P5V_RF_PG_GPIO_Port, P5V_RF_PG_Pin)) {
            disable_RF_PSU();
            enable_RF_PSU();
        }

        // (P3V3) RF_PG error fault detection
        while(isOff(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin)) {
            disable_RF_PSU();
            enable_RF_PSU();
        }
    }
}
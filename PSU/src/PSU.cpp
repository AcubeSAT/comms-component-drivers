#include "PSU.hpp"
#include "stm32h7xx_hal_gpio.h"



namespace PSU {
    void enable_FPGA_PSU() {
        HAL_GPIO_WritePin(P5V_FPGA_EN_GPIO_Port, P5V_FPGA_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }
    void disable_FPGA_PSU(){
        HAL_GPIO_WritePin(P5V_FPGA_EN_GPIO_Port, P5V_FPGA_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    void enable_RF_PSU() {
        HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }
    void disable_RF_PSU() {
        HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    bool PG_read() {
        GPIO_PinState pinstate_temporary[3];
        pinstate_temporary[0] = HAL_GPIO_ReadPin(P5V_FPGA_PG_GPIO_Port, P5V_FPGA_PG_Pin);
        pinstate_temporary[1] = HAL_GPIO_ReadPin(P5V_RF_PG_GPIO_Port, P5V_RF_PG_Pin);
        pinstate_temporary[2] = HAL_GPIO_ReadPin(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin);

        if (pinstate_temporary[0] == GPIO_PIN_SET
        && pinstate_temporary[1] == GPIO_PIN_SET
        && pinstate_temporary[2] == GPIO_PIN_SET)
            return true;
        else
            return false;
    }

    void solve_PG_fault() {
        GPIO_PinState pinstate_temporary[3];
        pinstate_temporary[0] = HAL_GPIO_ReadPin(P5V_FPGA_PG_GPIO_Port, P5V_FPGA_PG_Pin);
        pinstate_temporary[1] = HAL_GPIO_ReadPin(P5V_RF_PG_GPIO_Port, P5V_RF_PG_Pin);
        pinstate_temporary[2] = HAL_GPIO_ReadPin(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin);

        // P5V_FPGA_PG error fault detection
        while(pinstate_temporary[0] == GPIO_PIN_RESET){
            disable_FPGA_PSU();
            enable_FPGA_PSU();
            pinstate_temporary[0] = HAL_GPIO_ReadPin(P5V_FPGA_PG_GPIO_Port, P5V_FPGA_PG_Pin);
        }

        // (P5V OR P3V3) RF_PG error fault detection
        while(pinstate_temporary[1] == GPIO_PIN_RESET
        || pinstate_temporary[2] == GPIO_PIN_RESET) {
            disable_RF_PSU();
            enable_RF_PSU();
            pinstate_temporary[1] = HAL_GPIO_ReadPin(P5V_RF_PG_GPIO_Port, P5V_RF_PG_Pin);
            pinstate_temporary[2] = HAL_GPIO_ReadPin(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin);
        }
    }
}
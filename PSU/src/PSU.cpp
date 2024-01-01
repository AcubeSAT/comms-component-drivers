//
// Created by ntoylker on 27/12/2023.
//

#include "../inc/PSU.hpp"
#include "stm32h7xx_hal_gpio.h"



namespace PSU {

    /* WritePin() will take action to ENABLE the different parts of the PSU I suppose */
    /* PROBLEMS SO FAR:
     * Where the f are the definitions of 'RF_RST_GPIO_Port', 'P5V_FPGA_EN_GPIO_Port', etc...
     *
    */
    void enable_PSU_parts() {
        HAL_GPIO_WritePin(P5V_FPGA_EN_GPIO_Port, P5V_FPGA_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100); // γτ μπαινει η delay()?

        HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    bool PG_read() {
        GPIO_PinState pinstate_temporary[3];
        pinstate_temporary[0] = HAL_GPIO_ReadPin(P5V_RF_PG_GPIO_Port, P5V_FPGA_PG_Pin);
        pinstate_temporary[1] = HAL_GPIO_ReadPin(P5V_RF_PG_GPIO_Port, P5V_FPGA_PG_Pin);
        pinstate_temporary[2] =  HAL_GPIO_ReadPin(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin);

        if(pinstate_temporary[0] == GPIO_PIN_SET && pinstate_temporary[1] == GPIO_PIN_SET && pinstate_temporary[2] == GPIO_PIN_SET)
            return true;
        else
            return false;
    }

}
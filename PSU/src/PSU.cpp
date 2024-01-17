#include "PSU.hpp"
#include "stm32h7xx_hal_gpio.h"

namespace PSU {
    void PSU::enablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    void PSU::disablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    bool PSU::isPinOff(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
        return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET;
    }

    bool PSU::PGread() {
        if ( isPinOff(P5V_FPGA_PG_GPIO_Port, P5V_FPGA_PG_Pin)
        || isPinOff(P5V_RF_PG_GPIO_Port, P5V_RF_PG_Pin)
        || isPinOff(P3V3_RF_PG_GPIO_Port, P3V3_RF_PG_Pin) )
            return false;
        else
            return true;
    }

    void PSU::solvePGfault(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
        // hops into a loop until the PIN is ON
        while(isPinOff(GPIOx, GPIO_Pin)){
            disablePartPSU(GPIOx, GPIO_Pin);
            enablePartPSU(GPIOx, GPIO_Pin);
        }
    }
}
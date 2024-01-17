#include "PSU.hpp"
#include "stm32h7xx_hal_gpio.h"

namespace PSU {

    PSU::PSU(GPIO_TypeDef *p5vFPGApgPORT, uint16_t p5vFPGApgPIN, GPIO_TypeDef *p5vRFpgPORT, uint16_t p5vRFpgPIN,
             GPIO_TypeDef *p3v3RFenPORT, uint16_t p3v3RFpgPIN, GPIO_TypeDef *p5vFPGAenPORT, uint16_t p5vFPGAenPIN,
             GPIO_TypeDef *p5vRFenPORT, uint16_t p5vRFenPIN) {
        this->p5vFPGApgPORT = p5vFPGApgPORT;
        this->p5vFPGApgPIN = p5vFPGApgPIN;

        this->p5vRFpgPORT = p5vRFpgPORT;
        this->p5vRFpgPIN = p5vRFpgPIN;

        this->p3v3RFenPORT = p3v3RFenPORT;
        this->p3v3RFpgPIN = p3v3RFpgPIN;

        this->p5vFPGAenPORT = p5vFPGAenPORT;
        this->p5vFPGAenPIN = p5vFPGAenPIN;

        this->p5vRFenPORT = p5vRFenPORT;
        this->p5vRFenPIN = p5vRFenPIN;
    }

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
        if ( isPinOff(p5vFPGApgPORT, p5vFPGApgPIN)
             || isPinOff(p5vRFpgPORT, p5vRFpgPIN)
             || isPinOff(p3v3RFenPORT, p3v3RFpgPIN) )
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
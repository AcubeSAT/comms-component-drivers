#include "PSU.hpp"
#include "stm32h7xx_hal_gpio.h"


PSU::PSU(GPIO_TypeDef *p5vFPGApgPORTx, uint16_t p5vFPGApgPINx, GPIO_TypeDef *p5vRFpgPORTx, uint16_t p5vRFpgPINx,
         GPIO_TypeDef *p3v3RFenPORTx, uint16_t p3v3RFpgPINx, GPIO_TypeDef *p5vFPGAenPORTx, uint16_t p5vFPGAenPINx,
         GPIO_TypeDef *p5vRFenPORTx, uint16_t p5vRFenPINx) {
    p5vFPGApgPORT = p5vFPGApgPORTx;
    p5vFPGApgPIN = p5vFPGApgPINx;

    p5vRFpgPORT = p5vRFpgPORTx;
    p5vRFpgPIN = p5vRFpgPINx;

    p3v3RFenPORT = p3v3RFenPORTx;
    p3v3RFpgPIN = p3v3RFpgPINx;

    p5vFPGAenPORT = p5vFPGAenPORTx;
    p5vFPGAenPIN = p5vFPGAenPINx;

    p5vRFenPORT = p5vRFenPORTx;
    p5vRFenPIN = p5vRFenPINx;
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

#pragma once

#include <optional>
#include <cstdint>
#include "main.h"

class PSU {
    public:

        GPIO_TypeDef *p5vFPGApgPORT;
        uint16_t p5vFPGApgPIN;

        GPIO_TypeDef *p5vRFpgPORT;
        uint16_t p5vRFpgPIN;

        GPIO_TypeDef *p3v3RFpgPORT;
        uint16_t p3v3RFpgPIN;

        GPIO_TypeDef *p5vFPGAenPORT;
        uint16_t p5vFPGAenPIN;

        GPIO_TypeDef *p5vRFenPORT;
        uint16_t p5vRFenPIN;
        /**
         * @param p5vFPGApgPORT  P5V_FPGA_PG_PORT
         * @param p5vFPGApgPIN P5V_FPGA_PG_PIN
         *
         * @param p5vRFpgPORT P5V_RF_PG_PORT
         * @param p5vRFpgPIN P5V_RF_PG_PIN
         *
         * @param p3v3RFpgPORT P3V3_RF_PG_PORT
         * @param p3v3RFpgPIN P3V3_RF_PG_PIN
         *
         * @param p5vFPGAenPORT P5V_FPGA_EN_PORT
         * @param p5vFPGAenPIN P5V_FPGA_EN_PIN
         *
         * @param p5vRFenPORT P5V_RF_EN_PORT
         * @param p5vRFenPIN P5V_RF_EN_PIN
         *
         * @note For comms_eqm_software apply configuration AS IT IS BELOW:
         *      GPIOD, GPIO_PIN_9
         *    , GPIOC, GPIO_PIN_7
         *    , GPIOA, GPIO_PIN_9
         *    , GPIOD, GPIO_PIN_8
         *    , GPIOE, GPIO_PIN_13
         */
        PSU(GPIO_TypeDef *p5vFPGApgPORTx, uint16_t p5vFPGApgPINx, GPIO_TypeDef *p5vRFpgPORTx, uint16_t p5vRFpgPINx,
            GPIO_TypeDef *p3v3RFpgPORTx, uint16_t p3v3RFpgPINx, GPIO_TypeDef *p5vFPGAenPORTx, uint16_t p5vFPGAenPINx,
            GPIO_TypeDef *p5vRFenPORTx, uint16_t p5vRFenPINx);


        /**
         * @brief Checks whether the given Pin is OFF (RESET STATE)
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         * @return True if Pin is OFF, otherwise False
         */
        bool isPinOff(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /**
         * @brief Sets given Pin to state SET
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         */
        void enablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /**
         * @brief Resets given Pin to state RESET
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         */
        void disablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /**
         * @brief Reads the Power-Good (PG) signals from 3 pins:
         * P5V_FPGA_PG pin PD9,
         * P5V_RF_PG pin PC7,
         * P3V3_RF_PG Pin PA9,
         * @return True if all 3 Pins are in SET state, otherwise False
         */
        bool PGread();

        /**
         * @brief Solves the 'NO Power-Good Signal' error (PG signals went on RESET state) of the given PIN
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         */
        void solvePGfault(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
};

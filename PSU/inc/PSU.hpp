#pragma once

#include <optional>
#include <cstdint>
#include "main.h"

namespace PSU {

    class PSU {
    public:
        PSU(void) {};

        /**
         * @brief Checks whether the given Pin is OFF (RESET STATE)
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         * @return True if Pin is OFF, otherwise False
         */
        static bool isPinOff(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /**
         * @brief Sets given Pin to state SET
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         */
        static void enablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /**
         * @brief Resets given Pin to state RESET
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         */
        static void disablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /**
         * @brief Reads the Power-Good (PG) signals from 3 pins:
         * P5V_FPGA_PG pin PD9,
         * P5V_RF_PG pin PC7,
         * P3V3_RF_PG Pin PA9,
         * @return True if all 3 Pins are in SET state, otherwise False
         */
        static bool PGread();

        /**
         * @brief Solves the 'NO Power-Good Signal' error (PG signals went on RESET state) of the given PIN
         * @param GPIOx: Where x can be (A..K) to select the GPIO peripheral.
         * @param GPIO_Pin: specifies the port bit to be written.
         *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
         */
        static void solvePGfault(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    };
}
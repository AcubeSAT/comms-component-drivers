#pragma once

#include <optional>
#include <cstdint>
#include "main.h"
//#include "etl/utility.h"

// creating a 'namespace' in order to not conflict variables' names with other coders
namespace PSU {

    class PSU {
    public:
        PSU(void) {};

        static bool isOff(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /* Sets given Pin to state SET */
        static void enablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
        /* Resets given Pin to state RESET */
        static void disablePartPSU(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

        /* Reads the Power-Good (PG) signals from 3 pins:
          P5V_FPGA_PG pin PD9
          P5V_RF_PG pin PC7
          P3V3_RF_PG Pin PA9
        and returns 'true' if all 3 are in SET state or else 'false' */
        static bool PG_read();

        /* */
        static void solvePGfault(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    };
}

//#endif //STM32H7A3ZIQSETUP_PSU_HPP
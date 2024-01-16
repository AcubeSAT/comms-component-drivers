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

        /* Sets P5V_FPGA_EN_Pin PD8 to state SET */
        static void enable_FPGA_PSU();
        /* Resets P5V_FPGA_EN_Pin PD8 to state RESET */
        static void disable_FPGA_PSU();

        /* Sets P5V_RF_EN pin PE13 to state SET */
        static void enable_RF_PSU();
        /* Resets P5V_RF_EN pin PE13 to state RESET */
        static void disable_RF_PSU();

        /* Reads the Power-Good (PG) signals from 3 pins:
          P5V_FPGA_PG pin PD9
          P5V_RF_PG pin PC7
          P3V3_RF_PG Pin PA9
        and returns 'true' if all 3 are in SET state or else 'false' */
        static bool PG_read();

        /* Solves PG_fault error. If at least 1 pin out of the 3 returns RESET state,
        this function checks which pin/pins has/have the problem and steps in to set in SET state*/
        static void solve_PG_fault();
    };
}

//#endif //STM32H7A3ZIQSETUP_PSU_HPP
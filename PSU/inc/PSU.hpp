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

        static void enable_FPGA_PSU();
        static void disable_FPGA_PSU();
        static void enable_RF_PSU();
        static void disable_RF_PSU();

        static bool PG_read();

        static void solve_PG_fault();
    };
}

//#endif //STM32H7A3ZIQSETUP_PSU_HPP
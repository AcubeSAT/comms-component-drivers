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

        void enable_FPGA_PSU();
        void disable_FPGA_PSU();
        void enable_RF_PSU();
        void disable_RF_PSU();

        bool PG_read();

        void solve_PG_fault();
    };
}

//#endif //STM32H7A3ZIQSETUP_PSU_HPP
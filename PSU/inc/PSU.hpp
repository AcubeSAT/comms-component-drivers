//
// Created by ntoylker on 27/12/2023.
//

//#ifndef STM32H7A3ZIQSETUP_PSU_HPP
//#define STM32H7A3ZIQSETUP_PSU_HPP

#pragma once

#include <optional>
#include <cstdint>
#include "main.h"
//#include "etl/utility.h"

// creating a 'namespace' in order to not conflict variables' names with other coders
namespace PSU {

    class PSU {

        PSU(void) {};

        void enable_PSU_parts() ;
        bool PG_read() ;
    };
}

//#endif //STM32H7A3ZIQSETUP_PSU_HPP
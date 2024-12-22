#pragma once

#include "stm32h7xx_hal.h"

namespace AC7Z020 {
    enum class SPIChain {
        UHF_TM,
        UHF_TC,
        SBAND_TM
    };

    /*
     * Packet lengths according to DDJF_TTC in bytes.
     */
    enum PacketLength : uint16_t {
        SBandPacketLength = 648,
        UHFTMPacketLength = 1032,
        UHFTCPacketLength = 146
    };

}
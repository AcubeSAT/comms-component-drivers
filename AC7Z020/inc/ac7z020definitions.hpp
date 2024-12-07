#pragma once

namespace AC7Z020 {
    enum class SPIChain {
        UHF_TM,
        UHF_TC,
        SBAND_TM
    }
    /*
     * Packet lengths according to DDJF_TTC.
     */
    enum class PacketLength : constexpr static uint16_t {
            SBandPacketLength = 648,
            UHFTMPacketLength = 1032,
            UHFTCPacketLength = 146
    };
}
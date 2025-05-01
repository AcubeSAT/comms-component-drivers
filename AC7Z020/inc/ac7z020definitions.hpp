#pragma once

#include "stm32h7xx_hal.h"

namespace AC7Z020 {
    // TODO these register addresses and irq codes must match with the FPGA
    enum RegisterAddress : uint16_t {
      IRQ = 0x00,                // 1 byte register
      SBAND_FRAME = 0x01,
      UHF_TM_FRAME = 0x02,
      UHF_TC_FRAME = 0x03,
      UHF_TC_FRAME_LEN = 0x04,   // 2 bytes register
    };

    /**
     * Specified in bytes. Lengths determined by DDJF_TT&C.
     * LEN_UHF_TC_FRAME corresponds to the maximum expected TC frame length.
     */
    enum FrameRegisterLength : uint16_t {
        LEN_SBAND_FRAME = 648,
        LEN_UHF_TM_FRAME = 1032,
        LEN_UHF_TC_FRAME = 146,
    };

    // TODO specify when the FULL_FRAME_TRANSMISSION_COMPLETE IRQs are sent.
    //      Looking at figure 6-4 of transceiver data sheet (EEC=1), there are 3 options:
    //      1. Right when I/Q samples are finished. The transceiver drivers wait for t_tx_proc_delay + t_pa_ramp
    //      2. Right when padding is finished. -//- for t_tx_proc_delay
    //      3. FPGA accounts for the above delays, by sending the IRQ exactly t_tx_proc_delay + t_pa_ramp time after
    //         IQ samples are finished
    enum class IrqCode : uint8_t {
      UHF_PREAMBLE_RECEIVED = 0x01,    // used for knowing when to freeze the AGC
      UHF_FULL_FRAME_RECEIVED = 0x02,  // used for knowing when to release the AGC and read the frame from the FPGA
      UHF_FULL_FRAME_TRANSMISSION_COMPLETE = 0x03,    // confirmation about successful transmission
      SBAND_FULL_FRAME_TRANSMISSION_COMPLETE = 0x04,
    };
}
#ifndef STM32H7A3ZIQSETUP_AT86RF215CUSTOMCONFIG_HPP
#define STM32H7A3ZIQSETUP_AT86RF215CUSTOMCONFIG_HPP

#include "at86rf215defaultConfig.hpp"

/**
 *  Contains custom configuration
 */
namespace AT86RF215{
    constexpr static uint32_t FrequencyUHF = 436500;
    constexpr static uint32_t FrequencyS = 2425000;

struct AT86RF215CustomConfiguration : public AT86RF215DefaultConfiguration{
    AT86RF215CustomConfiguration() { // Add custom settings here
        // Crystal Oscillator
        fastStartUp = false;

        // PLL
        // Frequencies are configures using Fine Resolution Channel Scheme CNM.CM=1 (section 6.3.2)
        pllFrequency09 = ((FrequencyUHF - 377000) * 65536 / 6500) >> 8;
        pllChannelNumber09 = ((FrequencyUHF - 377000) * 65536 / 6500) & 0xFF;
        pllChannelMode09 = PLLChannelMode::FineResolution450;

        pllFrequency24 = ((FrequencyS - 377000) * 65536 / 6500) >> 8;
        pllChannelNumber24 = ((FrequencyS - 377000) * 65536 / 6500) & 0xFF;
        pllChannelMode24 = PLLChannelMode::FineResolution2443;

        channelSpacing09 =  0x30;

        // TX Front-end
        powerAmplifierRampTime09 = PowerAmplifierRampTime::RF_PARAMP32U;
        transmitterCutOffFrequency09 = TransmitterCutOffFrequency::RF_FLC80KHZ;
        txRelativeCutoffFrequency09 = TxRelativeCutoffFrequency::FCUT_025;
        transceiverSampleRate09 =  TransmitterSampleRate::FS_400;
        txOutPower09 = 0x0;

        // RX Front-end
        rxBandwidth09 = ReceiverBandwidth::RF_BW200KHZ_IF250KHZ;
        rxRelativeCutoffFrequency09 = RxRelativeCutoffFrequency::FCUT_0375;
        receiverSampleRate09 = ReceiverSampleRate::FS_400;  // Do not change this
        automaticGainControlTarget24 = AutomaticGainTarget::DB42;
        gainControlWord09 = 0x23;

        // Baseband Core
        frameCheckSequenceFilter09 = false;
        frameCheckSequenceFilter24 = false;
        transmitterAutoFrameCheckSequence09 = false;
        transmitterAutoFrameCheckSequence24 = false;
        physicalLayerType09 = PhysicalLayerType::BB_MRFSK;
        physicalLayerType24 = PhysicalLayerType::BB_MRFSK;

        // Baseband IRQ
        // All interrupts enabled
        frameBufferLevelIndication09 = true;
        frameBufferLevelIndication24 = true;
        agcRelease09 = true;
        agcRelease24 = true;
        agcHold09 = true;
        agcHold24 = true;
        transmitterFrameEnd09 = true;
        transmitterFrameEnd24 = true;
        receiverExtendedMatch09 = true;
        receiverExtendedMatch24 = true;
        receiverAddressMatch09 = true;
        receiverAddressMatch24 = true;
        receiverFrameEnd09 = true;
        receiverFrameEnd24 = true;
        receiverFrameStart09 = true;
        receiverFrameStart24 = true;

        // Radio IRQ
        iqIfSynchronizationFailure09 = true;
        iqIfSynchronizationFailure24 = true;
        trasnceiverError09 = true;
        trasnceiverError24 = true;
        batteryLow09 = true;
        batteryLow24 = true;
        energyDetectionCompletion09 = true;
        energyDetectionCompletion24 = true;
        transceiverReady09 = true;
        transceiverReady24 = true;

    }
};

}

#endif /*STM32H7A3ZIQSETUP_AT86RF215CUSTOMCONFIG_HPP*/

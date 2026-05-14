/**
 * @file at86rf215Config.hpp
 *
 * @brief This file contains objects that hold the configuration of the transceiver. They are used to configure
 *        the transceiver upon calling the chipReset() function, and should be considered the single source of truth.
 *
 * @details Any api function that needs to modify the configuration must do so temporarily, restoring the original one.
 *          These configuration objects contain a "default configuration", which is set for the needs of the AcubeSAT
 *          mission.
 */

#pragma once
#include "at86rf215Definitions.hpp"

namespace AT86RF215 {
    /// Define here how long the transceiver should wait for certain events, before throwing an error (milliseconds).
    static constexpr uint16_t SpiAccessMutexTimeoutMs              = 100;
    static constexpr uint16_t SpiByteWriteCompleteDelayMs          = 100;
    static constexpr uint16_t SpiByteReadCompleteDelayMs           = 100;
    static constexpr uint16_t Radio09AccessMutexDelayMs            = 100;
    static constexpr uint16_t Radio24AccessMutexDelayMs            = 100;
    static constexpr uint16_t IqTxInterfaceAccessMutexDelayMs      = 100;
    static constexpr uint16_t IqPacketReception09DelayMs           = 100;
    static constexpr uint16_t IqPacketReception24DelayMs           = 100;
    static constexpr uint16_t BasebandCorePacketReception09DelayMs = 100;
    static constexpr uint16_t BasebandCorePacketReception24DelayMs = 100;
    static constexpr uint16_t TransceiverReadyDelayMs              = 100;
    static constexpr uint16_t BasebandTx09DelayMs                  = 100;
    static constexpr uint16_t BasebandTx24DelayMs                  = 100;
    static constexpr uint16_t EnergyDetCompletion09DelayMs         = 100;
    static constexpr uint16_t EnergyDetCompletion24DelayMs         = 100;

    struct RXConfig {
        // RFn_RXBWC
        ReceiverBandwidth receiverBandwidth09, receiverBandwidth24;
        bool ifInversion09, ifInversion24;
        bool ifShift09, ifShift24;
        // RFn_RXDFE
        RxRelativeCutoffFrequency rxRelativeCutoffFrequency09, rxRelativeCutoffFrequency24;
        ReceiverSampleRate receiverSampleRate09, receiverSampleRate24;
        /// RFn_AGCC RFn_AGCS
        bool agcInput09, agcInput24;
        AverageTimeNumberSamples averageTimeNumberSamples09, averageTimeNumberSamples24;
        AGCReset agcReset_09, agcReset_24;
        AGCFreezeControl agcFreezeControl_09, agcFreezeControl_24;
        AGCEnable agcEnabled09, agcEnabled24;
        AutomaticGainTarget automaticGainTarget09, automaticGainTarget24;
        uint8_t gainControlWord09, gainControlWord24;
        /// RFn_EDC RFn_RDD
        EnergyDetectionMode energyDetectionMode09, energyDetectionMode24;
        uint8_t energyDetectDurationFactor09, energyDetectDurationFactor24;
        EnergyDetectionTimeBasis energyDetectionBasis09, energyDetectionBasis24;
        static RXConfig defaultRXConfig() {
            return {
                    /// RFn_RXBWC
                    .receiverBandwidth09 = ReceiverBandwidth::RF_BW160KHZ_IF250KHZ,
                    .receiverBandwidth24 = ReceiverBandwidth::RF_BW1000KHZ_IF1000KHZ,
                    .ifInversion09 = false,
                    .ifInversion24 = false,
                    .ifShift09 = false,
                    .ifShift24 = false,
                    /// RFn_RXDFE
                    .rxRelativeCutoffFrequency09 = RxRelativeCutoffFrequency::FCUT_0375,
                    .rxRelativeCutoffFrequency24 = RxRelativeCutoffFrequency::FCUT_1,
                    .receiverSampleRate09 = ReceiverSampleRate::FS_4000,
                    .receiverSampleRate24 = ReceiverSampleRate::FS_4000_3,
                    /// RFn_AGCC
                    .agcInput09 = false,
                    .agcInput24 = false,
                    .averageTimeNumberSamples09 = AverageTimeNumberSamples::AVGS_16,
                    .averageTimeNumberSamples24 = AverageTimeNumberSamples::AVGS_8,
                    .agcReset_09 = AGCReset::default_agc_reset,
                    .agcReset_24 = AGCReset::default_agc_reset,
                    .agcFreezeControl_09 = AGCFreezeControl::no_freeze,
                    .agcFreezeControl_24 = AGCFreezeControl::no_freeze,
                    .agcEnabled09 = AGCEnable::agc_enabled,
                    .agcEnabled24 = AGCEnable::agc_enabled,
                    /// RF_AGCSx
                    .automaticGainTarget09 = AutomaticGainTarget::DB30,
                    .automaticGainTarget24 = AutomaticGainTarget::DB30,
                    /// Maximum Receive Gain
                    .gainControlWord09 = 23,
                    .gainControlWord24 = 23,
                    /// RFn_EDC // RFn_EDD //
                    .energyDetectionMode09 = EnergyDetectionMode::RF_EDAUTO,
                    .energyDetectionMode24 = EnergyDetectionMode::RF_EDAUTO,
                    .energyDetectDurationFactor09 = 0x16,
                    .energyDetectDurationFactor24 = 0x16,
                    .energyDetectionBasis09 = EnergyDetectionTimeBasis::RF_8MS,
                    .energyDetectionBasis24 = EnergyDetectionTimeBasis::RF_8MS,
            };
        }

        // update params
        void setRxbwc(Transceiver transceiver, ReceiverBandwidth bw, bool inversion, bool shift) {
            if (transceiver == Transceiver::RF09) {
                receiverBandwidth09 = bw;
                ifInversion09 = inversion;
                ifShift09 = shift;
            }
            else {
                receiverBandwidth24 = bw;
                ifInversion24 = inversion;
                ifShift24 = shift;
            }
        }

        //
        void setRxdfe(Transceiver transceiver, RxRelativeCutoffFrequency cutoff, ReceiverSampleRate sampleRate) {
            if (transceiver == Transceiver::RF09) {
                rxRelativeCutoffFrequency09 = cutoff;
                receiverSampleRate09 = sampleRate;
            }
            else {
                rxRelativeCutoffFrequency24 = cutoff;
                receiverSampleRate24 = sampleRate;
            }
        }

        //
        void setEdc(Transceiver transceiver, EnergyDetectionTimeBasis timeBasis, EnergyDetectionMode mode,
                    uint8_t detectFactor) {
            if (transceiver == Transceiver::RF09) {
                energyDetectionBasis09 = timeBasis;
                energyDetectionMode09 = mode;
                energyDetectDurationFactor09 = detectFactor;
            }
            else {
                energyDetectionBasis24 = timeBasis;
                energyDetectionMode24 = mode;
                energyDetectDurationFactor24 = detectFactor;
            }
        }

        //
        void setAgcc(Transceiver transceiver, bool input, AverageTimeNumberSamples avgSamples, AGCEnable enabled,
                     AutomaticGainTarget target) {
            if (transceiver == Transceiver::RF09) {
                agcInput09 = input;
                averageTimeNumberSamples09 = avgSamples;
                agcEnabled09 = enabled;
                automaticGainTarget09 = target;
            }
            else {
                agcInput24 = input;
                averageTimeNumberSamples24 = avgSamples;
                agcEnabled24 = enabled;
                automaticGainTarget24 = target;
            }
        }
    };

    struct TXConfig {
        // RFn_TXDFE
        TxRelativeCutoffFrequency txRelativeCutoffFrequency09, txRelativeCutoffFrequency24;
        DirectModEnableFSKDM directModulation09, directModulation24;
        TransmitterSampleRate transceiverSampleRate09, transceiverSampleRate24;
        // RFn_TXCUTC
        PowerAmplifierRampTime powerAmplifierRampTime09, powerAmplifierRampTime24;
        TransmitterCutOffFrequency transmitterCutOffFrequency09, transmitterCutOffFrequency24;
        // RFn_PAC
        PowerAmplifierCurrentControl powerAmplifierCurrentControl09, powerAmplifierCurrentControl24;
        uint8_t txOutPower09, txOutPower24;
        // RFn_TXDACI
        bool enableInputI09, enableInputI24;
        uint8_t dataInputI09, dataInputI24;
        // RFn_TXDACQ
        bool enableInputQ09, enableInputQ24;
        uint8_t dataInputQ09, dataInputQ24;

        static TXConfig defaultTXConfig() {
            return {
                    // RFn_TXDFE
                    .txRelativeCutoffFrequency09 = TxRelativeCutoffFrequency::FCUT_0375,
                    .txRelativeCutoffFrequency24 = TxRelativeCutoffFrequency::FCUT_1,
                    .directModulation09 = DirectModEnableFSKDM::direct_mod_enabled,
                    .directModulation24 = DirectModEnableFSKDM::direct_mod_disabled,
                    .transceiverSampleRate09 = TransmitterSampleRate::FS_400,
                    .transceiverSampleRate24 = TransmitterSampleRate::FS_4000_3,
                    // RFn_TXCUTC
                    /*TT&C (for sband) -> pass band bw is 1000kHz, so we need half for baseband. try something like 500-625 kHz.
                     * the external low pass filter does not cover us, since it has cutoff freq of 2500 MHz,and we need something
                     * around 2425 + 0.5 MHz*/
                    .powerAmplifierRampTime09 = PowerAmplifierRampTime::RF_PARAMP4U,
                    .powerAmplifierRampTime24 = PowerAmplifierRampTime::RF_PARAMP4U,
                    .transmitterCutOffFrequency09 = TransmitterCutOffFrequency::RF_FLC100KHZ,
                    .transmitterCutOffFrequency24 = TransmitterCutOffFrequency::RF_FLC1000KHZ,
                    // RF_n_PAC
                    .powerAmplifierCurrentControl09 = PowerAmplifierCurrentControl::PA_NO,
                    .powerAmplifierCurrentControl24 = PowerAmplifierCurrentControl::PA_NO,
                    .txOutPower09 = 0x1F,
                    .txOutPower24 = 0x00,
                    // RFn_TXDACI
                    .enableInputI09 = false,
                    .enableInputI24 = false,
                    .dataInputI09 = 0x7E, // maximum amplitude
                    .dataInputI24 = 0x3F, // zero
                    // RFn_TXDACQ
                    .enableInputQ09 = false,
                    .enableInputQ24 = false,
                    .dataInputQ09 = 0x7E, // maximum amplitude
                    .dataInputQ24 = 0x3F, // zero
            };
        }
        void setTxdfe(Transceiver transceiver, TxRelativeCutoffFrequency cutoffFrequency, DirectModEnableFSKDM modulation, TransmitterSampleRate sampleRate) {
            if (transceiver == Transceiver::RF09) {
                txRelativeCutoffFrequency09 = cutoffFrequency;
                directModulation09 = modulation;
                transceiverSampleRate09 = sampleRate;
            }
            else {
                txRelativeCutoffFrequency24 = cutoffFrequency;
                directModulation24 = modulation;
                transceiverSampleRate24 = sampleRate;
            }
        }
        void setTxcutc(Transceiver transceiver, PowerAmplifierRampTime rampTime, TransmitterCutOffFrequency cutoffFrequency) {
            if (transceiver == Transceiver::RF09) {
                powerAmplifierRampTime09 = rampTime;
                transmitterCutOffFrequency09 = cutoffFrequency;
            }
            else {
                powerAmplifierRampTime24 = rampTime;
                transmitterCutOffFrequency24 = cutoffFrequency;
            }
        }
        void setRfnpac(Transceiver transceiver, PowerAmplifierCurrentControl currentControl, uint8_t outPower) {
            if (transceiver == Transceiver::RF09) {
                powerAmplifierCurrentControl09 = currentControl;
                txOutPower09 = outPower;
            }
            else {
                powerAmplifierCurrentControl24 = currentControl;
                txOutPower24 = outPower;
            }
        }
        void setRfndaci(Transceiver transceiver, bool enableInputI, uint8_t dataInputI) {
            if (transceiver == Transceiver::RF09) {
                enableInputI09 = enableInputI;
                dataInputI09 = dataInputI;
            }
            else {
                enableInputI24 = enableInputI;
                dataInputI24 = dataInputI;
            }
        }
        void setRfndacq(Transceiver transceiver, bool enableInputQ, uint8_t dataInputQ) {
            if (transceiver == Transceiver::RF09) {
                enableInputQ09 = enableInputQ;
                dataInputQ09 = dataInputQ;
            }
            else {
                enableInputQ24 = enableInputQ;
                dataInputQ24 = dataInputQ;
            }
        }
    };

    struct BasebandCoreConfig {
        // BBCn_PC
        bool continuousTransmit09, continuousTransmit24;
        bool frameCheckSequenceFilterEn09, frameCheckSequenceFilterEn24;
        bool transmitterAutoFrameCheckSequence09, transmitterAutoFrameCheckSequence24;
        FrameCheckSequenceType frameCheckSequenceType09, frameCheckSequenceType24;
        bool baseBandEnable09, baseBandEnable24;
        PhysicalLayerType physicalLayerType09, physicalLayerType24;
        /// BBCn_FSKCO
        BandwidthTimeProduct bandwidth_time_09, bandwidth_time_24;
        ModIndexScale midxs_09, midxs_24;
        ModIndex midx_09, midx_24;
        FskModOrder mord_09, mord_24;
        /// BBCn_FSKC1
        FreqInversion freq_inv_09, freq_inv_24;
        MrFskSymbolRate sr_09, sr_24;
        /// BBCn_FSKC2
        PreambleDetection preamble_detection_09, preamble_detection_24;
        ReceiverOverride receiver_override_09, receiver_override_24;
        ReceiverPreambleTimeout receiver_preamble_timeout_09, receiver_preamble_timeout_24;
        ModeSwitchEnable mode_switch_en_09, mode_switch_en_24;
        PreambleInversion preamble_inversion_09, preamble_inversion_24;
        FecScheme fec_scheme_09, fec_scheme_24;
        InterleavingEnable interleaving_enable_09, interleaving_enable_24;
        /// BBCn_FSKC3
        SfdDetectionThreshold sfdt_09, sfdt_24;
        PreambleDetectionThreshold prdt_09, prdt_24;
        /// BBCn_FSKC4
        SfdQuantization sfdQuantization_09, sfdQuantization_24;
        Sfd32 sfd32_09, sfd32_24;
        RawModeReversalBit rawModeReversalBit_09, rawModeReversalBit_24;
        CSFD1 csfd1_09, csfd1_24;
        CSFD0 csfd0_09, csfd0_24;
        /// BBCn_FSKPHRTX
        SfdUsed sfdUsed_09, sfdUsed_24;
        DataWhitening dataWhitening_09, dataWhitening_24;
        /// BBCn_FSKDM
        FskPreamphasisEnable fskPreamphasisEnable_09, fskPreamphasisEnable_24;
        DirectModEnableFSKDM directModEnableFskdm_09, directModEnableFskdm_24;
        /// BBCn_FSKPLL
        uint16_t fskPreambleLengthOctets_09, fskPreambleLengthOctets_24;

        static BasebandCoreConfig defaultBasebandCoreConfig() {
            return {
                    /// BBCn_PC
                    .continuousTransmit09 = false,
                    .continuousTransmit24 = false,
                    .frameCheckSequenceFilterEn09 = false,
                    .frameCheckSequenceFilterEn24 = true,
                    .transmitterAutoFrameCheckSequence09 = true,
                    .transmitterAutoFrameCheckSequence24 = true,
                    .frameCheckSequenceType09 = FrameCheckSequenceType::FCS_32,
                    .frameCheckSequenceType24 = FrameCheckSequenceType::FCS_32,
                    .baseBandEnable09 = false,
                    .baseBandEnable24 = false,
                    .physicalLayerType09 = PhysicalLayerType::BB_MRFSK,
                    .physicalLayerType24 = PhysicalLayerType::BB_OFF,
                    /// BBCn_FSKC0
                    .bandwidth_time_09 = BandwidthTimeProduct::BT_1_0,
                    .bandwidth_time_24 = BandwidthTimeProduct::BT_2_0,
                    .midxs_09 = ModIndexScale::s_1_0,
                    .midxs_24 = ModIndexScale::s_1_0,
                    .midx_09 = ModIndex::bf_1_000,
                    .midx_24 = ModIndex::bf_1_000,
                    .mord_09 = FskModOrder::binary_fsk,
                    .mord_24 = FskModOrder::binary_fsk,
                    /// BBCn_FSKC1
                    .freq_inv_09 = FreqInversion::freq_inversion_off,
                    .freq_inv_24 = FreqInversion::freq_inversion_off,
                    .sr_09 = MrFskSymbolRate::sr_50,
                    .sr_24 = MrFskSymbolRate::sr_50,
                    /// BBCn_FSKC2
                    .preamble_detection_09 = PreambleDetection::preamble_det_without_rssi,
                    .preamble_detection_24 = PreambleDetection::preamble_det_without_rssi,
                    .receiver_override_09 = ReceiverOverride::restart_by_18db_stronger_frame,
                    .receiver_override_24 = ReceiverOverride::restart_by_18db_stronger_frame,
                    .receiver_preamble_timeout_09 = ReceiverPreambleTimeout::timeout_disabled,
                    .receiver_preamble_timeout_24 = ReceiverPreambleTimeout::timeout_disabled,
                    .mode_switch_en_09 = ModeSwitchEnable::disabled,
                    .mode_switch_en_24 = ModeSwitchEnable::disabled,
                    .preamble_inversion_09 = PreambleInversion::no_inversion,
                    .preamble_inversion_24 = PreambleInversion::no_inversion,
                    .fec_scheme_09 = FecScheme::NRNSC,
                    .fec_scheme_24 = FecScheme::NRNSC,
                    .interleaving_enable_09 = InterleavingEnable::enabled,
                    .interleaving_enable_24 = InterleavingEnable::enabled,
                    /// BBCn_FSKC3
                    .sfdt_09 = SfdDetectionThreshold::default_sfd_IEEE,
                    .sfdt_24 = SfdDetectionThreshold::default_sfd_IEEE,
                    .prdt_09 = PreambleDetectionThreshold::increased_preamble_sensitivity,
                    .prdt_24 = PreambleDetectionThreshold::default_value,
                    /// BBCn_FSC4
                    .sfdQuantization_09 = SfdQuantization::SOFT_DECISION,
                    .sfdQuantization_24 = SfdQuantization::SOFT_DECISION,
                    .sfd32_09 = Sfd32::TWO_16BIT_SFD,
                    .sfd32_24 = Sfd32::TWO_16BIT_SFD,
                    .rawModeReversalBit_09 = RawModeReversalBit::MSB_FIRST,
                    .rawModeReversalBit_24 = RawModeReversalBit::MSB_FIRST,
                    .csfd1_09 = CSFD1::UNCODED_IEEE_MODE,
                    .csfd1_24 = CSFD1::UNCODED_IEEE_MODE,
                    .csfd0_09 = CSFD0::UNCODED_IEEE_MODE,
                    .csfd0_24 = CSFD0::UNCODED_IEEE_MODE,
                    /// BBCn_FSKPHRTX
                    .sfdUsed_09 = SfdUsed::sfd0_used,
                    .sfdUsed_24 = SfdUsed::sfd0_used,
                    .dataWhitening_09 = DataWhitening::psdu_data_whitening_disabled,
                    .dataWhitening_24 = DataWhitening::psdu_data_whitening_enabled,
                    /// BBCn_FSKDM
                    .fskPreamphasisEnable_09 = FskPreamphasisEnable::preamphasis_disabled,
                    .fskPreamphasisEnable_24 = FskPreamphasisEnable::preamphasis_disabled,
                    .directModEnableFskdm_09 = DirectModEnableFSKDM::direct_mod_enabled,
                    .directModEnableFskdm_24 = DirectModEnableFSKDM::direct_mod_disabled,
                    /// BBCn_FSKPLL
                    .fskPreambleLengthOctets_09 = 8,
                    .fskPreambleLengthOctets_24 = 8,
            };
        }
        /// BBC_PC
        void setBbcPc(Transceiver transceiver, bool ct, bool fcsfEn, bool tautoFcs, FrameCheckSequenceType fcsType, bool bbEn, PhysicalLayerType plType) {
            if (transceiver == Transceiver::RF09) {
                continuousTransmit09 = ct;
                frameCheckSequenceFilterEn09 = fcsfEn;
                transmitterAutoFrameCheckSequence09 = tautoFcs;
                frameCheckSequenceType09 = fcsType;
                baseBandEnable09 = bbEn;
                physicalLayerType09 = plType;
            }
            else {
                continuousTransmit24 = ct;
                frameCheckSequenceFilterEn24 = fcsfEn;
                transmitterAutoFrameCheckSequence24 = tautoFcs;
                frameCheckSequenceType24 = fcsType;
                baseBandEnable24 = bbEn;
                physicalLayerType24 = plType;
            }
        }
        /// BBC_FSKC0
        void setBbcFskc0(Transceiver transceiver, BandwidthTimeProduct bwTime, ModIndexScale midxs,
                          ModIndex midx, FskModOrder mord) {
            if (transceiver == Transceiver::RF09) {
                bandwidth_time_09 = bwTime;
                midxs_09 = midxs;
                midx_09 = midx;
                mord_09 = mord;
            }
            else {
                bandwidth_time_24 = bwTime;
                midxs_24 = midxs;
                midx_24 = midx;
                mord_24 = mord;
            }
        }
        /// BBC_FSKC1
        void setBbcFskc1(Transceiver transceiver, FreqInversion freqInv, MrFskSymbolRate sr) {
            if (transceiver == Transceiver::RF09) {
                freq_inv_09 = freqInv;
                sr_09 = sr;
            }
            else {
                freq_inv_24 = freqInv;
                sr_24 = sr;
            }
        }
        /// BBC_FSKC2
        void setBbcFskc2(Transceiver transceiver, PreambleDetection preambleDet, ReceiverOverride recOverride,
                          ReceiverPreambleTimeout recPreambleTimeout, ModeSwitchEnable modeSwitchEn,
                          PreambleInversion preambleInv, FecScheme fecScheme,
                          InterleavingEnable interleavingEn) {
            if (transceiver == Transceiver::RF09) {
                preamble_detection_09 = preambleDet;
                receiver_override_09 = recOverride;
                receiver_preamble_timeout_09 = recPreambleTimeout;
                mode_switch_en_09 = modeSwitchEn;
                preamble_inversion_09 = preambleInv;
                fec_scheme_09 = fecScheme;
                interleaving_enable_09 = interleavingEn;
            }
            else {
                preamble_detection_24 = preambleDet;
                receiver_override_24 = recOverride;
                receiver_preamble_timeout_24 = recPreambleTimeout;
                mode_switch_en_24 = modeSwitchEn;
                preamble_inversion_24 = preambleInv;
                fec_scheme_24 = fecScheme;
                interleaving_enable_24 = interleavingEn;
            }
        }
        /// BBC_FSKC3
        void setBbcFskc3(Transceiver transceiver, SfdDetectionThreshold sfdDetectionThreshold, PreambleDetectionThreshold preambleDetectionThreshold) {
            if (transceiver == Transceiver::RF09) {
                sfdt_09 = sfdDetectionThreshold;
                prdt_09 = preambleDetectionThreshold;
            }
            else {
                sfdt_24 = sfdDetectionThreshold;
                prdt_24 = preambleDetectionThreshold;
            }
        }
        /// BBC_FSKC4
        void setBbcFskc4(Transceiver transceiver, SfdQuantization sfdQuantization, Sfd32 sfd32,
                          RawModeReversalBit rawModeReversalBit,
                          CSFD1 csfd1, CSFD0 csfd2) {
            if (transceiver == Transceiver::RF09) {
                sfdQuantization_09 = sfdQuantization;
                sfd32_09 = sfd32;
                rawModeReversalBit_09 = rawModeReversalBit;
                csfd1_09 = csfd1;
                csfd0_09 = csfd2;
            }
            else {
                sfdQuantization_24 = sfdQuantization;
                sfd32_24 = sfd32;
                rawModeReversalBit_24 = rawModeReversalBit;
                csfd1_24 = csfd1;
                csfd0_24 = csfd2;
            }
        }
        /// BBCn_FSKPHRTX
        void setBbcFskphrtx(Transceiver transceiver, SfdUsed sfdused, DataWhitening dataWhitening) {
            if (transceiver == Transceiver::RF09) {
                sfdUsed_09 = sfdused;
                dataWhitening_09 = dataWhitening;
            }
            else {
                sfdUsed_24 = sfdused;
                dataWhitening_24 = dataWhitening;
            }
        }
        /// BBCn_FSKDM
        void setBbcFskdm(Transceiver transceiver, FskPreamphasisEnable fskPreamphasisEnable, DirectModEnableFSKDM directModEnableFskdm) {
            if (transceiver == Transceiver::RF09) {
                fskPreamphasisEnable_09 = fskPreamphasisEnable;
                directModEnableFskdm_09 = directModEnableFskdm;
            }
            else {
                fskPreamphasisEnable_24 = fskPreamphasisEnable;
                directModEnableFskdm_24 = directModEnableFskdm;
            }
        }
    };

    struct FrequencySynthesizerConfig {
        /// This is a flag for setup(), so an error may be thrown in case the config was not initialized properly
        bool validConfig09, validConfig24;

        /// Cached frequency for easy access
        uint64_t frequency09, frequency24; // Frequency in kHz

        /// RFn_CNM
        PLLChannelMode channelMode09, channelMode24;
        /// RFn_PLL
        PLLBandwidth loopBandwidth09, loopBandwidth24;

        static FrequencySynthesizerConfig defaultFrequencySynthesizerConfig() {
            FrequencySynthesizerConfig fs;
            fs.setupFrequencySynthesizer(Transceiver::RF09, 436500, PLLChannelMode::FineResolution450, PLLBandwidth::BWDefault);
            fs.setupFrequencySynthesizer(Transceiver::RF24, 2425000, PLLChannelMode::FineResolution2443, PLLBandwidth::BWDefault);
            return fs;
        }

        // Frequency must be given in kHz. If the given frequency and channel mode are incompatible
        // with the given transceiver, the validConfig flag will be set to false.
        void setupFrequencySynthesizer(Transceiver transceiver, uint32_t frequency, PLLChannelMode channelMode, PLLBandwidth bw) {

            transceiver == Transceiver::RF09 ? (validConfig09 = false) : (validConfig24 = false);

            if (channelMode == PLLChannelMode::IEECompliant) {
                // @TODO: CCF0 and CS for each band in 68d, 68e tables of IEEE Std 802.15.4g™-2012
                return;
            }
            else if (channelMode == PLLChannelMode::FineResolution450) {
                if (frequency < 389500 || frequency > 510000 || transceiver == Transceiver::RF24) {
                    return;
                }
                frequency09 = frequency;
                channelMode09 = channelMode;
                loopBandwidth09 = bw;
            }
            else if (channelMode == PLLChannelMode::FineResolution900) {
                if (frequency < 779000 || frequency > 1020000 || transceiver == Transceiver::RF24) {
                    return;
                }
                frequency09 = frequency;
                channelMode09 = channelMode;
                loopBandwidth09 = bw;
            }
            else {
                if (frequency < 2400000 || frequency > 2483500 || transceiver == Transceiver::RF09) {
                    return;
                }
                frequency24 = frequency;
                channelMode24 = channelMode;
                loopBandwidth24 = bw;
            }

            transceiver == Transceiver::RF09 ? (validConfig09 = true) : (validConfig24 = true);
        }

    };

    struct ExternalFrontEndConfig {
        /// RFn_AUXS
        ExternalLNABypass externalLNABypass09, externalLNABypass24;
        AutomaticGainControlMAP automaticGainControlMAP09, automaticGainControlMAP24;
        AnalogVoltageEnable analogVoltageEnable09, analogVoltageEnable24;
        AutomaticVoltageExternal automaticVoltageExternal09, automaticVoltageExternal24;
        PowerAmplifierVoltageControl powerAmplifierVoltageControl09, powerAmplifierVoltageControl24;
        /// RFn_PADFE
        ExternalFrontEndControl externalFrontEnd_09, externalFrontEnd_24;
        static ExternalFrontEndConfig defaultExternalFrontEndConfig() {
            return {
                    .externalLNABypass09 = ExternalLNABypass::FALSE,
                    .externalLNABypass24 = ExternalLNABypass::FALSE,
                    .automaticGainControlMAP09 = AutomaticGainControlMAP::INTERNAL_AGC,
                    .automaticGainControlMAP24 = AutomaticGainControlMAP::INTERNAL_AGC,
                    .analogVoltageEnable09 = AnalogVoltageEnable::ENABLED,
                    .analogVoltageEnable24 = AnalogVoltageEnable::ENABLED,
                    .automaticVoltageExternal09 = AutomaticVoltageExternal::DISABLED,
                    .automaticVoltageExternal24 = AutomaticVoltageExternal::DISABLED,
                    .powerAmplifierVoltageControl09 = PowerAmplifierVoltageControl::PAVC_2V4,
                    .powerAmplifierVoltageControl24 = PowerAmplifierVoltageControl::PAVC_2V4,
                    .externalFrontEnd_09 = ExternalFrontEndControl::front_end_config_txrx_switch,
                    .externalFrontEnd_24 = ExternalFrontEndControl::front_end_config_txrx_switch
                    };
        }
        void setRfnAuxs(
                Transceiver transceiver,
                ExternalLNABypass extLNA,            // externalLNABypass09
                AutomaticGainControlMAP agcMap,      // automaticGainControlMAP09
                AnalogVoltageEnable avEn,            // analogVoltageEnable09
                AutomaticVoltageExternal avExt,      // automaticVoltageExternal09
                PowerAmplifierVoltageControl pavCtrl // powerAmplifierVoltageControl09
        ) {
            if (transceiver == Transceiver::RF09) {
                externalLNABypass09 = extLNA;
                automaticGainControlMAP09 = agcMap;
                analogVoltageEnable09 = avEn;
                automaticVoltageExternal09 = avExt;
                powerAmplifierVoltageControl09 = pavCtrl;
            }
            else {
                externalLNABypass24 = extLNA;
                automaticGainControlMAP24 = agcMap;
                analogVoltageEnable24 = avEn;
                automaticVoltageExternal24 = avExt;
                powerAmplifierVoltageControl24 = pavCtrl;
            }
        }

        void setRfnPadfe(Transceiver transceiver, ExternalFrontEndControl externalFrontEndControl) {
            if (transceiver == Transceiver::RF09) {
                externalFrontEnd_09 = externalFrontEndControl;
            }
            else {
                externalFrontEnd_24 = externalFrontEndControl;
            }
        }
    };

    struct IQInterfaceConfig {
        // IQ Interface
        // RF_IQIFC0
        ExternalLoopback externalLoopback;
        IQOutputCurrent iqOutputCurrent;
        IQmodeVoltage iqmodeVoltage;
        IQmodeVoltageIEE iqmodeVoltageIEE;
        EmbeddedControlTX embeddedControlTX;
        // RF_IQIFC1
        ChipMode chipMode;
        SkewAlignment skewAlignment;

        static IQInterfaceConfig defaultIQInterfaceConfig() {
            return {
                    // RF_IQIFC0
                    .externalLoopback = ExternalLoopback::DISABLED,
                    .iqOutputCurrent = IQOutputCurrent::CURR_2_MA,
                    .iqmodeVoltage = IQmodeVoltage::MODE_150_MV,
                    .iqmodeVoltageIEE = IQmodeVoltageIEE::IEEE,
                    .embeddedControlTX = EmbeddedControlTX::ENABLED,
                    // RF_IQIFC1
                    .chipMode = ChipMode::RF_MODE_BBRF24, // baseband mode on sub GHz transceiver (debugging)
                    .skewAlignment = SkewAlignment::SKEW3906NS};
        }

        void setRfIqfc0(
                ExternalLoopback external_Loopback,
                IQOutputCurrent iqOutput_Current,
                IQmodeVoltage iqmode_Voltage,
                IQmodeVoltageIEE iqmodeVoltage_IEE,
                EmbeddedControlTX embeddedControl_Tx
        ) {

            externalLoopback = external_Loopback;
            iqOutputCurrent = iqOutput_Current;
            iqmodeVoltage = iqmode_Voltage;
            iqmodeVoltageIEE = iqmodeVoltage_IEE;
            embeddedControlTX = embeddedControl_Tx;
        }
        void setRfIqifc1(ChipMode chip_Mode, SkewAlignment skew_alignment) {
            chipMode = chip_Mode;
            skewAlignment = skew_alignment;
        }
    };

    struct BasebandCoreInterruptsConfig {
        /// BBCn_IRQM
        bool frameBufferLevelIndication09, frameBufferLevelIndication24;
        bool agcRelease09, agcRelease24;
        bool agcHold09, agcHold24;
        bool transmitterFrameEnd09, transmitterFrameEnd24;
        bool receiverExtendedMatch09, receiverExtendedMatch24;
        bool receiverAddressMatch09, receiverAddressMatch24;
        bool receiverFrameEnd09, receiverFrameEnd24;
        bool receiverFrameStart09, receiverFrameStart24;

        static BasebandCoreInterruptsConfig defaultBasebandCoreInterruptsConfig() {
            return {
                    .frameBufferLevelIndication09 = true,
                    .frameBufferLevelIndication24 = true,
                    .agcRelease09 = true,
                    .agcRelease24 = true,
                    .agcHold09 = true,
                    .agcHold24 = true,
                    .transmitterFrameEnd09 = true,
                    .transmitterFrameEnd24 = true,
                    .receiverExtendedMatch09 = true,
                    .receiverExtendedMatch24 = true,
                    .receiverAddressMatch09 = true,
                    .receiverAddressMatch24 = true,
                    .receiverFrameEnd09 = true,
                    .receiverFrameEnd24 = true,
                    .receiverFrameStart09 = true,
                    .receiverFrameStart24 = true,
            };
        }
        void setupInterruptsConfig(Transceiver transceiver,
                                   bool fbl,
                                   bool ar,
                                   bool ah,
                                   bool tfe,
                                   bool rem,
                                   bool ram,
                                   bool rfe,
                                   bool rfs) {
            if (transceiver == Transceiver::RF09) {
                frameBufferLevelIndication09 = fbl;
                agcRelease09 = ar;
                agcHold09 = ah;
                transmitterFrameEnd09 = tfe;
                receiverExtendedMatch09 = rem;
                receiverAddressMatch09 = ram;
                receiverFrameEnd09 = rfe;
                receiverFrameStart09 = rfs;
            }
            else {
                frameBufferLevelIndication24 = fbl;
                agcRelease24 = ar;
                agcHold24 = ah;
                transmitterFrameEnd24 = tfe;
                receiverExtendedMatch24 = rem;
                receiverAddressMatch24 = ram;
                receiverFrameEnd24 = rfe;
                receiverFrameStart24 = rfs;
            }
        }
    };

    struct RadioInterruptsConfig {
        // RFn_IRQM
        bool iqIfSynchronizationFailure09, iqIfSynchronizationFailure24;
        bool transceiverError09, transceiverError24;
        bool batteryLow09, batteryLow24;
        bool energyDetectionCompletion09, energyDetectionCompletion24;
        bool transceiverReady09, transceiverReady24;
        bool wakeup09, wakeup24;

        static RadioInterruptsConfig defaultRadioInterruptsConfig() {
            return {
                    // RFn_IRQM
                    .iqIfSynchronizationFailure09 = true,
                    .iqIfSynchronizationFailure24 = true,
                    .transceiverError09 = true,
                    .transceiverError24 = true,
                    .batteryLow09 = true,
                    .batteryLow24 = true,
                    .energyDetectionCompletion09 = true,
                    .energyDetectionCompletion24 = true,
                    .transceiverReady09 = true,
                    .transceiverReady24 = true,
                    .wakeup09 = true,
                    .wakeup24 = true,
            };
        }
        // Setup function to configure values
        void setupRadioInterruptsConfig(Transceiver transceiver,
                                        bool syncFail,
                                        bool txErr,
                                        bool batLow,
                                        bool edComp,
                                        bool txReady,
                                        bool wake) {
            if (transceiver == Transceiver::RF09) {
                iqIfSynchronizationFailure09 = syncFail;
                transceiverError09 = txErr;
                batteryLow09 = batLow;
                energyDetectionCompletion09 = edComp;
                transceiverReady09 = txReady;
                wakeup09 = wake;
            }
            else {
                iqIfSynchronizationFailure24 = syncFail;
                transceiverError24 = txErr;
                batteryLow24 = batLow;
                energyDetectionCompletion24 = edComp;
                transceiverReady24 = txReady;
                wakeup24 = wake;
            }
        }
    };

    struct GeneralConfiguration {

        /// RF_CFG
        bool irqMaskMode;
        IRQPolarity irqPolarity;
        PadDriverStrength padDriverStrength;

        /// RF_BMDVC
        /// Generation of an interrupt if supply voltage (EVDD) drops below the configured threshold level
        BatteryMonitorVoltageThreshold batteryMonitorVoltage;
        BatteryMonitorHighRange batteryMonitorHighRange;

        /// Crystal oscillator - RF_XOC
        CrystalTrim crystalTrim;
        bool fastStartUp;

        static struct GeneralConfiguration defaultGeneralConfig() {
            return {
                    .irqMaskMode = true,
                    .irqPolarity = IRQPolarity::ACTIVE_HIGH,
                    .padDriverStrength = PadDriverStrength::RF_DRV4,

                    .batteryMonitorVoltage = BatteryMonitorVoltageThreshold::BMHR_270_180,
                    .batteryMonitorHighRange = BatteryMonitorHighRange::LOW_RANGE,

                    .crystalTrim = CrystalTrim::TRIM_00,
                    .fastStartUp = false
            };
        }

        void setupRfCfg(bool irq_mask_mode, IRQPolarity irq_polarity, PadDriverStrength pad_driver_strength) {
            irqMaskMode = irq_mask_mode;
            irqPolarity = irq_polarity;
            padDriverStrength = pad_driver_strength;
        }

        void setupRfBmdvc(BatteryMonitorVoltageThreshold battery_monitor_voltage_threshold, BatteryMonitorHighRange battery_monitor_high_range) {
            batteryMonitorVoltage = battery_monitor_voltage_threshold;
            batteryMonitorHighRange = battery_monitor_high_range;
        }

        void setupRdXoc(CrystalTrim crystal_trim, bool fast_startup) {
            crystalTrim = crystal_trim;
            fastStartUp = fast_startup;
        }
    };
} // namespace AT86RF215
#ifndef INC_AT86RF215CONFIG_HPP_
#define INC_AT86RF215CONFIG_HPP_

#include "at86rf215definitions.hpp"

/**
 *  Contains default register configuration
 */
namespace AT86RF215 {

struct AT86RF215DefaultConfiguration {
	ReceiverEnergyDetectionAveragingDuration receiverEnergyDetectionAveragingDuration =
			ReceiverEnergyDetectionAveragingDuration::REDAD_8U;
	ReceiverBandwidth receiverBandwidth =
			ReceiverBandwidth::RF_BW2000KHZ_IF2000KHZ;
//	RxRelativeCutoffFrequency rxRelativeCutoffFrequency =
//			RxRelativeCutoffFrequency::FCUT_025;
    RxRelativeCutoffFrequency rxRelativeCutoffFrequency =
            RxRelativeCutoffFrequency::FCUT_0375;

	ReceiverSampleRate receiverSampleRate = ReceiverSampleRate::FS_4000;
	HighRateLegacyOQPSK highRateLegacyOQPSK = HighRateLegacyOQPSK::HDRL_DISABLED;
	SFDSearchSpace sfdSearchSpace = SFDSearchSpace::NSFD1;
	RXSpuriousCompensation rxSpuriousCompensation =
			RXSpuriousCompensation::DISABLED;
	ReducePowerConsumption reducePowerConsumption =
			ReducePowerConsumption::DISABLED;
	EnableProprietaryModes enableProprietaryModes =
			EnableProprietaryModes::NOT_SUPPORTED;
	FCSType fcsType = FCSType::FCS32;
	ReceiveMode receiveMode = ReceiveMode::OQPSKONLY;
	OQPSKPulseShapingFilter oqpskPulseShapingFilter =
			OQPSKPulseShapingFilter::BB_RC08;
	OQPSKChipFrequency oqpskChipFrequency = OQPSKChipFrequency::BB_FCHIP100;
	RXOOverride rxoOverride = RXOOverride::ENABLED;
	RXOLEGOverride rxoLEGOverride = RXOLEGOverride::ENABLED;

	// Battery Monitor
	BatteryMonitorVoltage batteryMonitorVoltage =
			BatteryMonitorVoltage::BMHR_270_180;
	BatteryMonitorHighRange batteryMonitorHighRange =
			BatteryMonitorHighRange::LOW_RANGE;

	// Crystal oscillator
	CrystalTrim crystalTrim = CrystalTrim::TRIM_00;
	bool fastStartUp = true;

	// RSSI
	EnergyDetectionTimeBasis energyDetectionBasis09 =
			EnergyDetectionTimeBasis::RF_8MS;
	EnergyDetectionTimeBasis energyDetectionBasis24 =
			EnergyDetectionTimeBasis::RF_8MS;
	EnergyDetectionMode energyDetectionMode09 = EnergyDetectionMode::RF_EDAUTO;
	EnergyDetectionMode energyDetectionMode24 = EnergyDetectionMode::RF_EDAUTO;
	uint8_t energyDetectFactor09 = 0x10;
	uint8_t energyDetectFactor24 = 0x10;

	// PLL
	uint16_t pllFrequency09 = 0x8CF8;
	uint16_t pllFrequency24 = 0x8CF8;
	uint16_t pllChannelNumber09 = 0x0000;
	uint16_t pllChannelNumber24 = 0x0000;
	PLLChannelMode pllChannelMode09 = PLLChannelMode::IEECompliant;
	PLLChannelMode pllChannelMode24 = PLLChannelMode::IEECompliant;
	PLLBandwidth pllBandwidth09 = PLLBandwidth::BWDefault;
	PLLBandwidth pllBandwidth24 = PLLBandwidth::BWDefault;
	uint8_t channelSpacing09 =  0x08;
	uint8_t channelSpacing24 =  0x08;


	// TX Front-end
	PowerAmplifierRampTime powerAmplifierRampTime09 =
			PowerAmplifierRampTime::RF_PARAMP4U;
	PowerAmplifierRampTime powerAmplifierRampTime24 =
			PowerAmplifierRampTime::RF_PARAMP4U;
	TransmitterCutOffFrequency transmitterCutOffFrequency09 =
			TransmitterCutOffFrequency::RF_FLC500KHZ;
	TransmitterCutOffFrequency transmitterCutOffFrequency24 =
			TransmitterCutOffFrequency::RF_FLC500KHZ;
	TxRelativeCutoffFrequency txRelativeCutoffFrequency09 =
			TxRelativeCutoffFrequency::FCUT_025;
	TxRelativeCutoffFrequency txRelativeCutoffFrequency24 =
			TxRelativeCutoffFrequency::FCUT_025;
	bool directModulation09 = false;                  // Direct modulation must be kept off
	bool directModulation24 = false;
	TransmitterSampleRate transceiverSampleRate09 =
			TransmitterSampleRate::FS_4000;
	TransmitterSampleRate transceiverSampleRate24 =
			TransmitterSampleRate::FS_4000;
	PowerAmplifierCurrentControl powerAmplifierCurrentControl09 =
			PowerAmplifierCurrentControl::PA_NO;
	PowerAmplifierCurrentControl powerAmplifierCurrentControl24 =
			PowerAmplifierCurrentControl::PA_NO;
	uint8_t txOutPower09 = 0x1F;
	uint8_t txOutPower24 = 0x1F;
	ExternalLNABypass externalLNABypass09 = ExternalLNABypass::FALSE;
	ExternalLNABypass externalLNABypass24 = ExternalLNABypass::FALSE;
	AutomaticGainControlMAP automaticGainControlMAP09 =
			AutomaticGainControlMAP::INTERNAL_AGC;
	AutomaticGainControlMAP automaticGainControlMAP24 =
			AutomaticGainControlMAP::INTERNAL_AGC;
	AnalogVoltageEnable analogVoltageEnable09 = AnalogVoltageEnable::DISABLED;
	AnalogVoltageEnable analogVoltageEnable24 = AnalogVoltageEnable::DISABLED;
	AutomaticVoltageExternal automaticVoltageExternal09 =
			AutomaticVoltageExternal::DISABLED;
	AutomaticVoltageExternal automaticVoltageExternal24 =
			AutomaticVoltageExternal::DISABLED;
	PowerAmplifierVoltageControl powerAmplifierVoltageControl09 =
			PowerAmplifierVoltageControl::PAVC_2V4;
	PowerAmplifierVoltageControl powerAmplifierVoltageControl24 =
			PowerAmplifierVoltageControl::PAVC_2V4;

	// RX Front-end
	bool ifInversion09 = false;
	bool ifInversion24 = false;
	bool ifShift09 = false;
	bool ifShift24 = false;
	ReceiverBandwidth rxBandwidth09 = ReceiverBandwidth::RF_BW2000KHZ_IF2000KHZ;
	ReceiverBandwidth rxBandwidth24 = ReceiverBandwidth::RF_BW2000KHZ_IF2000KHZ;
	RxRelativeCutoffFrequency rxRelativeCutoffFrequency09 =
			RxRelativeCutoffFrequency::FCUT_025;
	RxRelativeCutoffFrequency rxRelativeCutoffFrequency24 =
			RxRelativeCutoffFrequency::FCUT_025;
	ReceiverSampleRate receiverSampleRate09 = ReceiverSampleRate::FS_4000;
	ReceiverSampleRate receiverSampleRate24 = ReceiverSampleRate::FS_4000;
	bool agcInput09 = false;
	bool agcInput24 = false;
	AverageTimeNumberSamples averageTimeNumberSamples09 =
			AverageTimeNumberSamples::AVGS_8;
	AverageTimeNumberSamples averageTimeNumberSamples24 =
			AverageTimeNumberSamples::AVGS_8;
	bool agcEnabled09 = true;
	bool agcEnabled24 = true;
	AutomaticGainTarget automaticGainControlTarget09 = AutomaticGainTarget::DB30;
	AutomaticGainTarget automaticGainControlTarget24 = AutomaticGainTarget::DB30;
	uint8_t gainControlWord09 = 0x17;
	uint8_t gainControlWord24 = 0x17;

	// IQ Interface
	ExternalLoopback externalLoopback = ExternalLoopback::DISABLED;
	IQOutputCurrent iqOutputCurrent = IQOutputCurrent::CURR_2_MA;
	IQmodeVoltage iqmodeVoltage = IQmodeVoltage::MODE_200_MV;
	IQmodeVoltageIEE iqmodeVoltageIEE = IQmodeVoltageIEE::CMV;
	EmbeddedControlTX embeddedControlTX = EmbeddedControlTX::DISABLED;
	ChipMode chipMode = ChipMode::RF_MODE_BBRF;
	SkewAlignment skewAlignment = SkewAlignment::SKEW3906NS;

	// Baseband Core

	bool continuousTransmit09 = false;
	bool continuousTransmit24 = false;
	bool frameCheckSequenceFilter09 = true;
	bool frameCheckSequenceFilter24 = true;
	bool transmitterAutoFrameCheckSequence09 = true;
	bool transmitterAutoFrameCheckSequence24 = true;
	FrameCheckSequenceType frameCheckSequenceType09 = FrameCheckSequenceType::FCS_32;
	FrameCheckSequenceType frameCheckSequenceType24 = FrameCheckSequenceType::FCS_32;
	bool baseBandEnable09 = true;
	bool baseBandEnable24 = true;
	PhysicalLayerType physicalLayerType09 = PhysicalLayerType::BB_OFF;
	PhysicalLayerType physicalLayerType24 = PhysicalLayerType::BB_OFF;
    bool enablePE09 = false;      // Relevant when direct modulation is on. Keep them false.
    bool enablePE24 = false;
    uint8_t configPE0_09 = 0x00;  // preemphasis filter settings (table 6-57)
    uint8_t configPE1_09 = 0x00;
    uint8_t configPE2_09 = 0x00;
    uint8_t configPE0_24 = 0x00;
    uint8_t configPE1_24 = 0x00;
    uint8_t configPE2_24 = 0x00;

    // Enabled Interrupts

	// Baseband IRQ
    // All interrupts enabled
	bool frameBufferLevelIndication09 = false;
	bool frameBufferLevelIndication24 = false;
	bool agcRelease09 = false;
	bool agcRelease24 = false;
	bool agcHold09 = false;
	bool agcHold24 = false;
	bool transmitterFrameEnd09 = false;
	bool transmitterFrameEnd24 = false;
	bool receiverExtendedMatch09 = false;
	bool receiverExtendedMatch24 = false;
	bool receiverAddressMatch09 = false;
	bool receiverAddressMatch24 = false;
	bool receiverFrameEnd09 = false;
	bool receiverFrameEnd24 = false;
	bool receiverFrameStart09 = false;
	bool receiverFrameStart24 = false;

	// Radio IRQ
	bool iqIfSynchronizationFailure09 = false;
	bool iqIfSynchronizationFailure24 = false;
	bool trasnceiverError09 = false;
	bool trasnceiverError24 = false;
	bool batteryLow09 = false;
	bool batteryLow24 = false;
	bool energyDetectionCompletion09 = false;
	bool energyDetectionCompletion24 = false;
	bool transceiverReady09 = false;
	bool transceiverReady24 = false;
	bool wakeup09 = true;
	bool wakeup24 = true;

	// IRQ Config
	bool irqMaskMode = true;
	IRQPolarity irqPolarity = IRQPolarity::ACTIVE_HIGH;
	PadDriverStrength padDriverStrength = PadDriverStrength::RF_DRV4;
};

}

#endif /* INC_AT86RF215CONFIG_HPP_ */
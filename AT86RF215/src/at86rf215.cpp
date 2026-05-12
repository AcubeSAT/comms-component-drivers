#include "at86rf215.hpp"
#include "Task.hpp"
#include "at86rf215GuardUtilities.hpp"

namespace AT86RF215 {
    // definition
    AT86RF215Chip at86rf215Chip = AT86RF215Chip();

    /** =========== Driver's public interface  =========== **/
    etl::expected<void, Error> AT86RF215Chip::initializeResources(
        SPI_HandleTypeDef* spi_handle,
        GeneralConfiguration&& general_config,
        RXConfig&& rx_config,
        TXConfig&& tx_config,
        BasebandCoreConfig&& baseband_core_config,
        FrequencySynthesizerConfig&& frequency_synthesizer_config,
        ExternalFrontEndConfig&& external_front_end_config,
        BasebandCoreInterruptsConfig&& baseband_core_interrupts_config,
        RadioInterruptsConfig&& radio_interrupts_config,
        IQInterfaceConfig&& iq_interface_config) {
        hspi = spi_handle;
        receivedPacketLength09 = 0;
        receivedPacketLength24 = 0;

        // Initialize the mutexes and the event group
        spiAccessMutexHandle = xSemaphoreCreateMutexStatic(&spiAccessMutexBuffer);
        transceiver09MutexHandle = xSemaphoreCreateMutexStatic(&transceiver09MutexBuffer);
        transceiver24MutexHandle = xSemaphoreCreateMutexStatic(&transceiver24MutexBuffer);
        iqTxMutexHandle = xSemaphoreCreateMutexStatic(&iqTxMutexBuffer);
        eventGroupHandle = xEventGroupCreateStatic(&eventGroupBuffer);

        if (hspi == nullptr ||
            spiAccessMutexHandle == nullptr ||
            transceiver09MutexHandle == nullptr ||
            transceiver24MutexHandle == nullptr ||
            iqTxMutexHandle == nullptr ||
            eventGroupHandle == nullptr) {
            return etl::unexpected(Error::NULL_HANDLE);
        }

        // Set the configuration structures
        generalConfig = general_config;
        rxConfig = rx_config;
        txConfig = tx_config;
        basebandCoreConfig = baseband_core_config;
        freqSynthesizerConfig = frequency_synthesizer_config;
        externalFrontEndConfig = external_front_end_config;
        basebandCoreInterruptsConfig = baseband_core_interrupts_config;
        radioInterruptsConfig = radio_interrupts_config;
        iqInterfaceConfig = iq_interface_config;

        if (auto status = setStatePrivate(Transceiver::RF09, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }
        if (auto status = setStatePrivate(Transceiver::RF24, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }
        return {};
    }

    etl::expected<State, Error> AT86RF215Chip::getState(Transceiver transceiver) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }
        return getStatePrivate(transceiver);
    }

    etl::expected<void, Error> AT86RF215Chip::setState(Transceiver transceiver, State state_cmd) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) ||
            !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        if (auto status = setStatePrivate(transceiver, state_cmd); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::chipReset() {
        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockAll()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // Chip reset
        if (auto status = spiWrite8(RegisterAddress::RF_RST, 0x07); !status.has_value()) {
            return status;
        }

        // Wait for a very small amount of time to ensure the transceiver transitions to state RF_TRXOFF
        vTaskDelay(pdMS_TO_TICKS(2));

        // Reset IRQ status registers
        if (auto status = spiRead8(RegisterAddress::RF09_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (auto status = spiRead8(RegisterAddress::RF24_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (auto status = spiRead8(RegisterAddress::BBC0_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (auto status = spiRead8(RegisterAddress::BBC1_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // Restore the current config settings
        if (auto status = setStatePrivate(Transceiver::RF09, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }

        if (auto status = setStatePrivate(Transceiver::RF24, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }

        if (auto status = setup(); !setup().has_value()) {
            return status;
        }

        // reset event group
        xEventGroupClearBits(eventGroupHandle, AllEventBitsMask);
        basebandCoreIsReceiving09 = false;
        basebandCoreIsReceiving24 = false;
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::checkTransceiverConnection() {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        auto status = getPartNumber();
        if (!status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (status.value() != DevicePartNumber::AT86RF215 &&
            status.value() != DevicePartNumber::AT86RF215IQ &&
            status.value() != DevicePartNumber::AT86RF215M) {
            return etl::unexpected(Error::INVALID_REGISTER_VALUE);
        }

        return {};
    }

    etl::expected<int8_t, Error> AT86RF215Chip::singleShotEnergyMeasurement(
        Transceiver transceiver,
        etl::optional<ReceiverBandwidth> bw) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) ||
            !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        SingleShotMeasurementSetup singleShotMeasurementSetup(*this, transceiver, bw);
        if (auto status = singleShotMeasurementSetup.setup(); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // get to state tx prep
        if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        const IrqEventGroupBit transceiverReadyGroupBit =
            transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;
        if (auto status =
            waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // begin the single shot conversion
        bool bbcEnabled;
        RegisterAddress bbcPcReg;
        RegisterAddress edcReg;
        uint8_t bbcPcVal = 0;
        if (transceiver == Transceiver::RF09) {
            bbcEnabled = basebandCoreConfig.baseBandEnable09;
            bbcPcReg = RegisterAddress::BBC0_PC;
            edcReg = RegisterAddress::RF09_EDC;
        } else {
            bbcEnabled = basebandCoreConfig.baseBandEnable24;
            bbcPcReg = RegisterAddress::BBC1_PC;
            edcReg = RegisterAddress::RF24_EDC;
        }

        if (auto status = setStatePrivate(transceiver, State::RF_RX); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (auto status = spiWrite8(edcReg, static_cast<uint8_t>(EnergyDetectionMode::RF_EDSINGLE)); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // wait for the one shot measurement to finish
        const IrqEventGroupBit energyDetectionCompletionGroupBit =
            transceiver == Transceiver::RF09 ? IrqEventGroupBit::ENERGY_DETECTION_09_COMPLETE : IrqEventGroupBit::ENERGY_DETECTION_24_COMPLETE;
        const uint32_t energyDetectionCompletionGroupBitDelayMs =
            transceiver == Transceiver::RF09 ? EnergyDetCompletion09DelayMs : EnergyDetCompletion24DelayMs;

        if (auto status = waitForIrqEvent(mutexGuard, energyDetectionCompletionGroupBit, energyDetectionCompletionGroupBitDelayMs); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (auto status = setStatePrivate(transceiver, State::RF_TRXOFF); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        return getReceiverEnergyDetection(transceiver);
    }

    etl::expected<void, Error> AT86RF215Chip::transmitCarrier(Transceiver transceiver, uint32_t transmissionTimeMs) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) ||
            !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        DacOverrideSetup dacOverrideSetup(*this, transceiver);
        if (auto status = dacOverrideSetup.setup(); !status.has_value()) {
            return status;
        }

        // get to state tx prep
        const IrqEventGroupBit transceiverReadyGroupBit =
            transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;

        if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
            return status;
        }

        if (auto status = waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
            return status;
        }

        if (auto status = setStatePrivate(transceiver, State::RF_TX); !status.has_value()) {
            return status;
        }

        mutexGuard.unlockSpi();
        vTaskDelay(transmissionTimeMs);
        return {};
    }

    // TODO: Upon reaching RX state
    // wait 8μs + RXDFE.SR + Tu
    // read rssi
    etl::expected<void, Error> AT86RF215Chip::packetTransmissionBaseband(
        Transceiver transceiver,
        etl::span<uint8_t> packet) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        if (packet.size() > MaxBasebandCorePacketLength) {
            return etl::unexpected(Error::TX_BUFFER_TOO_LARGE);
        }

        // ensure valid chip mode
        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_RF ||                                              // no bb core is active
            (transceiver == Transceiver::RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09) ||    // 09 bb core is inactive
            (transceiver == Transceiver::RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ) {   // 24 bb core is inactive
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        // ensure the baseband core is active
        if ((transceiver == Transceiver::RF09 && !basebandCoreConfig.baseBandEnable09) ||
            (transceiver == Transceiver::RF24 && !basebandCoreConfig.baseBandEnable24)) {
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) ||
            !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        RegisterAddress regtxflh;
        RegisterAddress regtxfll;
        RegisterAddress regfbtxs;

        if (transceiver == Transceiver::RF09) {
            regtxflh = RegisterAddress::BBC0_TXFLH;
            regtxfll = RegisterAddress::BBC0_TXFLL;
            regfbtxs = RegisterAddress::BBC0_FBTXS;
        } else { // transceiver == RF24
            regtxflh = RegisterAddress::BBC1_TXFLH;
            regtxfll = RegisterAddress::BBC1_TXFLL;
            regfbtxs = RegisterAddress::BBC1_FBTXS;
        }

        // if (auto status = setStatePrivate(transceiver, State::RF_TRXOFF); !status.has_value()) {
        //     return etl::unexpected(status.error());
        // }

        // write length to register
        if (auto status = spiWrite8(regtxfll, packet.size() & 0xFF); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        if (auto status = spiWrite8(regtxflh, (packet.size() >> 8) & 0x07); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // write to tx frame buffer
        if (auto status = spiBlockWrite8(regfbtxs, packet); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // wait for the transceiver to enter RF_TXPREP (if it is not already in it)
        State currState;
        if (auto status = getStatePrivate(transceiver); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            currState = status.value();
        }

        if (currState != State::RF_TXPREP) {
            if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            const IrqEventGroupBit transceiverReadyGroupBit =
                transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;
            if (auto status = waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
                return status;
            }
        }

        // start tx
        if (auto status = setStatePrivate(transceiver, State::RF_TX); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // wait for the tx complete event, to ensure the operation
        // was completed
        const uint32_t basebandTxGroupBitDelayMs =
            transceiver == Transceiver::RF09 ? BasebandTx09DelayMs : BasebandTx24DelayMs;
        const IrqEventGroupBit basebandTxGroupBit =
            transceiver == Transceiver::RF09 ? IrqEventGroupBit::BASEBAND_TX_09_COMPLETE : IrqEventGroupBit::BASEBAND_TX_24_COMPLETE;

        if (auto status =
            waitForIrqEvent(mutexGuard, basebandTxGroupBit, basebandTxGroupBitDelayMs); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::preparePacketReceptionBaseband(
        Transceiver transceiver,
        etl::span<uint8_t> destBuff) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        if (destBuff.size() < MaxBasebandCorePacketLength) {
            return etl::unexpected(Error::DESTINATION_BUFFER_TOO_SMALL);
        }

        // ensure valid chip mode
        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_RF ||                                              // no bb core is active
            (transceiver == Transceiver::RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09) ||    // 09 bb core is inactive
            (transceiver == Transceiver::RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ) {   // 24 bb core is inactive
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) ||
            !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        IntBasebandCoreBasicModeSetup intBasebandCoreBasicModeSetup(*this, transceiver);
        if (auto status = intBasebandCoreBasicModeSetup.setup(); !status.has_value()) {
            return status;
        }

        // get to state tx prep
        if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        const IrqEventGroupBit transceiverReadyGroupBit =
            transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;
        if (auto status = waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
            return status;
        }

        if (transceiver == Transceiver::RF09) {
            destBuffer09 = destBuff;
        } else {
            destBuffer24 = destBuff;
        }

        // now set the state to rx
        //   clear possibly stale bit
        const IrqEventGroupBit basebandRxGroupBit =
            transceiver == Transceiver::RF09 ?  IrqEventGroupBit::BASEBAND_RX_09_COMPLETE : IrqEventGroupBit::BASEBAND_RX_24_COMPLETE;
        xEventGroupClearBits(eventGroupHandle, static_cast<uint32_t>(basebandRxGroupBit));

        if (auto status = setStatePrivate(transceiver, State::RF_RX); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        return {};
    }

    etl::expected<uint16_t, Error> AT86RF215Chip::waitForPacketReceptionBaseband(
        Transceiver transceiver,
        uint32_t timeoutDelayMs) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        } else {
            // The respective transceiver needs to be prepared again  if a synchronization was performed
            if (status.value() == true) {
                return etl::unexpected(Error::FAILED_DUE_TO_DESYNCHRONIZATION);
            }
        }

        // ensure valid chip mode
        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_RF ||                                              // no bb core is active
            (transceiver == Transceiver::RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09) ||    // 09 bb core is inactive
            (transceiver == Transceiver::RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ) {   // 24 bb core is inactive
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        // wait until a new packet is received (the actual packet copying is happening inside the interrupt)
        const auto basebandRxGroupBit =
            static_cast<uint32_t>(transceiver == Transceiver::RF09 ? IrqEventGroupBit::BASEBAND_RX_09_COMPLETE : IrqEventGroupBit::BASEBAND_RX_24_COMPLETE);
        if ((xEventGroupWaitBits(eventGroupHandle,
               basebandRxGroupBit,
               pdTRUE, pdFALSE,
               pdMS_TO_TICKS(timeoutDelayMs)) & basebandRxGroupBit) == false) {
            return etl::unexpected(Error::RX_WAIT_TIMEOUT);
        }

        // return the length
        return transceiver == Transceiver::RF09 ? receivedPacketLength09 : receivedPacketLength24;
    }

    etl::expected<void, Error> AT86RF215Chip::packetTransmissionIQEmbeddedControl(
        Transceiver transceiver,
        etl::delegate<bool()> basebandOp) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        if (iqInterfaceConfig.embeddedControlTX == EmbeddedControlTX::DISABLED) {
            return etl::unexpected(Error::EMBEDDED_CONTROL_DISABLED);
        }

        // ensure valid chip mode
        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF ||                                            // I/Q interface inactive
            (transceiver == Transceiver::RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ||    // 09 IQ IF  is inactive
            (transceiver == Transceiver::RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09) ) {   // 24 IQ IF  is inactive
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) || !mutexGuard.lockIqTx() || !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // set the requested radio to RF_TXPREP (if it is not already in it)
        State currState;
        if (auto status = getStatePrivate(transceiver); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            currState = status.value();
        }

        if (currState != State::RF_TXPREP) {
            if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            const IrqEventGroupBit transceiverReadyGroupBit =
                transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;
            if (auto status = waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
                return status;
            }
        }

        mutexGuard.unlockSpi();

        // Execute user's baseband operation (send packet to the I/Q interface of the transceiver)
        if (!basebandOp()) {
            return etl::unexpected(Error::BASEBAND_OPERATION_FUNCTION_FAILED);
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::preparePacketReceptionIQ(Transceiver transceiver) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        // ensure valid chip mode
        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF ||                                            // I/Q interface inactive
            (transceiver == Transceiver::RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ||    // 09 IQ IF  is inactive
            (transceiver == Transceiver::RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09) ) {   // 24 IQ IF  is inactive
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        // wait for the requested transceiver to become available and lock it
        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) || !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // wait for the transceiver to enter RF_TXPREP (if it is not already in it)
        State currState;
        if (auto status = getStatePrivate(transceiver); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            currState = status.value();
        }

        if (currState != State::RF_TXPREP) {
            if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            const IrqEventGroupBit transceiverReadyGroupBit =
                transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;
            if (auto status = waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
                return status;
            }
        }

        // now set the state to RX
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // clear possibly stale bits
        const auto iqPreambleReceptionGroupBit =
            static_cast<uint32_t>(transceiver == Transceiver::RF09 ? ExternalEventGroupBit::IQ_PREAMBLE_RECEPTION_09 : ExternalEventGroupBit::IQ_PREAMBLE_RECEPTION_24);
        const auto iqPacketReceptionGroupBit =
            static_cast<uint32_t>(transceiver == Transceiver::RF09 ? ExternalEventGroupBit::IQ_PACKET_RECEPTION_09 : ExternalEventGroupBit::IQ_PACKET_RECEPTION_24);
        xEventGroupClearBits(eventGroupHandle, iqPreambleReceptionGroupBit | iqPacketReceptionGroupBit);

        if (auto status = setStatePrivate(transceiver, State::RF_RX); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::waitForPacketReceptionIQ(Transceiver transceiver, uint32_t timeoutDelayMs) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        } else {
            // The respective transceiver needs to be prepared again
            if (status.value() == true) {
                return etl::unexpected(Error::FAILED_DUE_TO_DESYNCHRONIZATION);
            }
        }

        // ensure valid chip mode
        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF ||                                            // I/Q interface inactive
            (transceiver == Transceiver::RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ||    // 09 IQ IF  is inactive
            (transceiver == Transceiver::RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09) ) {   // 24 IQ IF  is inactive
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        // wait until a preamble is detected and lock the transceiver
        const auto iqPreambleReceptionGroupBit =
             static_cast<uint32_t>(transceiver == Transceiver::RF09 ? ExternalEventGroupBit::IQ_PREAMBLE_RECEPTION_09 : ExternalEventGroupBit::IQ_PREAMBLE_RECEPTION_24);
        const auto iqPacketReceptionGroupBit =
            static_cast<uint32_t>(transceiver == Transceiver::RF09 ? ExternalEventGroupBit::IQ_PACKET_RECEPTION_09 : ExternalEventGroupBit::IQ_PACKET_RECEPTION_24);

        if ((xEventGroupWaitBits(eventGroupHandle,
                            iqPreambleReceptionGroupBit,
                            pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(timeoutDelayMs)) & (iqPreambleReceptionGroupBit)) == false) {
            return etl::unexpected(Error::RX_WAIT_TIMEOUT);
        }

        // A frame is currently being received. Lock the transceiver while reception is being performed
        // and also freeze the agc.
        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) || !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        RegisterAddress agcc = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_AGCC : RegisterAddress::RF24_AGCC;
        if (auto status = spiApplyBitwiseOr(agcc, 0x02); !status.has_value()) {
            return status;
        }

        // wait for packet reception to finish
        mutexGuard.unlockSpi();
        uint32_t iqPacketReceptionDelayMs = transceiver == Transceiver::RF09 ? IqPacketReception09DelayMs : IqPacketReception24DelayMs;
        if ((xEventGroupWaitBits(eventGroupHandle,
        iqPacketReceptionGroupBit,
        pdTRUE, pdFALSE,
        pdMS_TO_TICKS(iqPacketReceptionDelayMs)) & iqPacketReceptionGroupBit) == false) {
            return etl::unexpected(Error::RECEPTION_FAILED);
        }

        // release the AGC
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        if (auto status = spiApplyBitwiseAnd(agcc, 0xFD); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::enableIQLoopbackMode() {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF){
            return etl::unexpected(Error::INVALID_CHIP_MODE);
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        if (auto status = spiApplyBitwiseOr(RegisterAddress::RF_IQIFC0, 0x80); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::disableIQLoopbackMode() {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        if (auto status = spiApplyBitwiseAnd(RegisterAddress::RF_IQIFC0, 0x7F); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::transmitMorseCodeOOK(
        Transceiver transceiver,
        float wpm,
        etl::string_view sequence) {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockTransceiver(transceiver) || !mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        DacOverrideSetup dacOverrideSetup(*this, transceiver);
        if (auto status = dacOverrideSetup.setup(); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // get to state tx prep
        if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        const IrqEventGroupBit transceiverReadyGroupBit =
            transceiver == Transceiver::RF09 ? IrqEventGroupBit::TRANSCEIVER_09_READY : IrqEventGroupBit::TRANSCEIVER_24_READY;
        if (auto status = waitForIrqEvent(mutexGuard, transceiverReadyGroupBit, TransceiverReadyDelayMs); !status.has_value()) {
            return status;
        }

        TickType_t lastWakeTimeTicks = xTaskGetTickCount();
        const TickType_t timeUnitTicks = pdMS_TO_TICKS(static_cast<uint16_t>(1200 / wpm));
        for (uint16_t i = 0; i < sequence.size(); i++) {
            if (sequence[i] == ' ') { // large delay for word gaps
                vTaskDelayUntil(&lastWakeTimeTicks, 7*timeUnitTicks);
                continue;
            }

            MorseCodeMapping morseCodeMapping = getMorse(sequence[i]);
            if (morseCodeMapping.dotDashNum == 0) { // skip unknown characters
                continue;
            }

            // transmit character
            for (uint8_t j = 0; j < morseCodeMapping.dotDashNum; j++) {
                if (j != 0) {
                    if (!mutexGuard.lockSpi()) {
                        return etl::unexpected(Error::MUTEX_LOCK_ERROR);
                    }
                }

                if (auto status = setStatePrivate(transceiver, State::RF_TX); !status.has_value()) {
                    return status;
                }

                mutexGuard.unlockSpi();

                if ((morseCodeMapping.dotDashMapping & (0x80 >> j)) == 0) { // dot
                    vTaskDelayUntil(&lastWakeTimeTicks, timeUnitTicks);
                } else { // dash
                    vTaskDelayUntil(&lastWakeTimeTicks, 3*timeUnitTicks);
                }

                if (!mutexGuard.lockSpi()) {
                    return etl::unexpected(Error::MUTEX_LOCK_ERROR);
                }

                if (auto status = setStatePrivate(transceiver, State::RF_TXPREP); !status.has_value()) {
                    return status;
                }

                mutexGuard.unlockSpi();

                // delay between character elements
                if (j != morseCodeMapping.dotDashNum - 1) {
                    vTaskDelayUntil(&lastWakeTimeTicks, timeUnitTicks);
                }
            }

            // Delay between characters (skip if next is a space)
            if (i != sequence.size() - 1 && sequence[i + 1] != ' ') {
                vTaskDelayUntil(&lastWakeTimeTicks, 3*timeUnitTicks);
            }
        }

        // clean up possibly stale transceiver ready event bit
        xEventGroupClearBits(eventGroupHandle, static_cast<uint32_t>(transceiverReadyGroupBit));
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::printState(Transceiver transceiver) {
        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        State rfState;
        if (auto status = getStatePrivate(transceiver); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            rfState = status.value();
        }

        switch (rfState) {
        case State::RF_NOP:
            LOG_DEBUG << "STATE: NOP";
            break;
        case State::RF_SLEEP:
            LOG_DEBUG << "STATE: SLEEP";
            break;
        case State::RF_TRXOFF:
            LOG_DEBUG << "STATE: TRXOFF";
            break;
        case State::RF_TX:
            LOG_DEBUG << "STATE: TX";
            break;
        case State::RF_RX:
            LOG_DEBUG << "STATE: RX";
            break;
        case State::RF_TRANSITION:
            LOG_DEBUG << "STATE: TRANSITION";
            break;
        case State::RF_RESET:
            LOG_DEBUG << "STATE: RESET";
            break;
        case State::RF_INVALID:
            LOG_DEBUG << "STATE: INVALID";
            break;
        case State::RF_TXPREP:
            LOG_DEBUG << "STATE: TXPREP";
            break;
        default:
            LOG_ERROR << "UNDEFINED";
            break;
        }

        return {};
    }

    void AT86RF215Chip::printError(Error& err) {
        switch (err) {
            case Error::FAILED_WRITING_TO_REGISTER:
                LOG_ERROR << "FAILED_WRITING_TO_REGISTER";
                break;
            case Error::FAILED_READING_FROM_REGISTER:
                LOG_ERROR << "FAILED_READING_FROM_REGISTER";
                break;
            case Error::FAILED_CHANGING_STATE:
                LOG_ERROR << "FAILED_CHANGING_STATE";
                break;
            case Error::UKNOWN_REQUESTED_STATE:
                LOG_ERROR << "UNKNOWN_REQUESTED_STATE";
                break;
            case Error::INVALID_TRANSCEIVER_FREQ:
                LOG_ERROR << "INVALID_TRANSCEIVER_FREQ";
                break;
            case Error::INVALID_STATE_FOR_OPERATION:
                LOG_ERROR << "INVALID_STATE_FOR_OPERATION";
                break;
            case Error::INVALID_PLL_CENTER_FREQ:
                LOG_ERROR << "INVALID_PLL_CENTER_FREQ";
                break;
            case Error::INVALID_RSSI_MEASUREMENT:
                LOG_ERROR << "INVALID_RSSI_MEASUREMENT";
                break;
            case Error::INVALID_AGC_CONTROl_WORD:
                LOG_ERROR << "INVALID_AGC_CONTROl_WORD";
                break;
            case Error::ONGOING_TRANSMISSION_RECEPTION:
                LOG_ERROR << "ONGOING_TRANSMISSION_RECEPTION";
                break;
            case Error::MUTEX_LOCK_ERROR:
                LOG_ERROR << "MUTEX_TIMEOUT";
                break;
            case Error::TRANSMISSION_FAILED:
                LOG_ERROR << "TRANSMISSION_FAILED";
                break;
            case Error::RECEPTION_FAILED:
                LOG_ERROR << "RECEPTION_FAILED";
                break;
            case Error::SINGLE_SHOT_ENERGY_MEASUREMENT_FAILED:
                LOG_ERROR << "SINGLE_SHOT_MEASUREMENT_FAILED";
                break;
            case Error::NULL_HANDLE:
                LOG_ERROR << "NULL_HANDLE";
                break;
            case Error::INVALID_CHIP_MODE:
                LOG_ERROR << "INVALID_CHIP_MODE";
                break;
            case Error::RX_WAIT_TIMEOUT:
                LOG_ERROR << "RX_WAIT_TIMEOUT";
                break;
            case Error::EMBEDDED_CONTROL_DISABLED:
                LOG_ERROR << "EMBEDDED_CONTROL_DISABLED";
                break;
            case Error::DESTINATION_BUFFER_TOO_SMALL:
                LOG_ERROR << "DESTINATION_BUFFER_TOO_SMALL";
                break;
            case Error::INVALID_REGISTER_VALUE:
                LOG_ERROR << "INVALID_REGISTER_VALUE";
                break;
            case Error::TX_BUFFER_TOO_LARGE:
                LOG_ERROR << "TX_BUFFER_TOO_LARGE";
                break;
            case Error::BASEBAND_OPERATION_FUNCTION_FAILED:
                LOG_ERROR << "BASEBAND_OPERATION_FUNCTION_FAILED";
                break;
            case Error::FAILED_DUE_TO_DESYNCHRONIZATION:
                LOG_ERROR << "FAILED_DUE_TO_DESYNCHRONIZATION";
                break;
            case Error::EVENT_WAIT_TIMEOUT:
                LOG_ERROR << "EVENT_WAIT_TIMEOUT";
                break;
            default:
                LOG_ERROR << "UNHANDLED_ERROR";
                break;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setDeepSleep() {
        if (auto status = synchronizeConfig(); !status.has_value() ) {
            return etl::unexpected(status.error());
        }

        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockAll()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // set both transceivers to state RF_TRXOFF first
        if (auto status = setStatePrivate(Transceiver::RF09, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }

        if (auto status = setStatePrivate(Transceiver::RF24, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }

        // set both transceivers to RF_SLEEP, in order to trigger DEEP_SLEEP
        if (auto status = setStatePrivate(Transceiver::RF09, State::RF_SLEEP); !status.has_value()) {
            return status;
        }

        if (auto status = setStatePrivate(Transceiver::RF24, State::RF_SLEEP); !status.has_value()) {
            return status;
        }

        // The transceiver loses its settings while in deep sleep, so configuration in now de-synchronized
        xEventGroupSetBits(eventGroupHandle, ConfigDesynchronizationGroupBit);
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::wakeFromDeepSleep() {
        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockAll()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // The command RF_TRXOFF needs to be written in one of the two transceivers only, as shown in fig 5-7
        if (auto status = setStatePrivate(Transceiver::RF09, State::RF_TRXOFF); !status.has_value()) {
            return status;
        }

        // Transition from DEEP_SLEEP to RF_TRXOFF may take up to 500us, wait longer and check if both transceivers are
        // woken up
        vTaskDelay(pdMS_TO_TICKS(5));

        if (auto status = getStatePrivate(Transceiver::RF09); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            if (status.value() != State::RF_TRXOFF) {
                return etl::unexpected(Error::INVALID_STATE_FOR_OPERATION);
            }
        }

        if (auto status = getStatePrivate(Transceiver::RF24); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            if (status.value() != State::RF_TRXOFF) {
                return etl::unexpected(Error::INVALID_STATE_FOR_OPERATION);
            }
        }

        return {};
    }

     etl::expected<IrqStatus, Error> AT86RF215Chip::handleIrq() {
        MutexGuard mutexGuard(*this);
        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        // Read all interrupt registers
        IrqStatus irqStatus;
        if (auto status = spiRead8(RegisterAddress::RF09_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            irqStatus.rf09IrqsStatus = status.value();
        }

        if (auto status = spiRead8(RegisterAddress::RF24_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            irqStatus.rf24IrqsStatus = status.value();
        }

        if (auto status = spiRead8(RegisterAddress::BBC0_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            irqStatus.bbc0IrqsStatus = status.value();
        }

        if (auto status = spiRead8(RegisterAddress::BBC1_IRQS); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            irqStatus.bbc1IrqsStatus = status.value();
        }

        /* Sub 1-GHz Transceiver */

        // Radio IRQ handling
        if ((irqStatus.rf09IrqsStatus.value() & InterruptMask::IFSynchronization) != 0) {
            // I/Q IF Synchronization Failure handling
        }
        if ((irqStatus.rf09IrqsStatus.value() & InterruptMask::TransceiverError) != 0) {
            // Transceiver Error handling
        }
        if ((irqStatus.rf09IrqsStatus.value() & InterruptMask::BatteryLow) != 0) {
            // Battery Low handling
        }
        if ((irqStatus.rf09IrqsStatus.value() & InterruptMask::EnergyDetectionCompletion) != 0) {
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::ENERGY_DETECTION_09_COMPLETE));
        }
        if ((irqStatus.rf09IrqsStatus.value() & InterruptMask::TransceiverReady) != 0) {
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::TRANSCEIVER_09_READY));
        }
        if ((irqStatus.rf09IrqsStatus.value() & InterruptMask::Wakeup) != 0) {
            // Wakeup handling
        }

        /// Baseband IRQ handling
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::FrameBufferLevelIndication) != 0) {
            // Frame Buffer Level Indication handling
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::AGCRelease) != 0) {
            // AGC Release handling
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::AGCHold) != 0) {
            // AGC Hold handling
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::TransmitterFrameEnd) != 0) {
            // notify packetTransmissionBaseband() about successful transmission
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::BASEBAND_TX_09_COMPLETE));
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::ReceiverExtendMatch) != 0) {
            // Receiver Extended Match handling
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::ReceiverAddressMatch) != 0) {
            // Receiver Address Match handling
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::ReceiverFrameStart) != 0) {
            // reception of frame started, do not allow the respective radio to be locked
            taskENTER_CRITICAL();
            basebandCoreIsReceiving09 = true;
            basebandCoreReceptionStartTime09 = xTaskGetTickCount();
            taskEXIT_CRITICAL();
        }
        if ((irqStatus.bbc0IrqsStatus.value() & InterruptMask::ReceiverFrameEnd) != 0) {
            if (auto status = getReceivedLength(Transceiver::RF09); !status.has_value()) {
                return etl::unexpected(status.error());
            } else {
                receivedPacketLength09 = status.value();
            }

            // Sanity check: Ensure received packet length fits in the destination buffer
            if (receivedPacketLength09 > destBuffer09.size()) {
                return etl::unexpected(Error::DESTINATION_BUFFER_TOO_SMALL);
            }

            if (auto status = spiBlockRead8(RegisterAddress::BBC0_FBRXS, destBuffer09.subspan(0, receivedPacketLength09)); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            // allow the respective radio to be locked again
            taskENTER_CRITICAL();
            basebandCoreIsReceiving09 = false;
            taskEXIT_CRITICAL();

            // notify waitForPacketReceptionBaseband()
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::BASEBAND_RX_09_COMPLETE));
        }

        /* 2.4 GHz Transceiver */

        // Radio IRQ handling
        if ((irqStatus.rf24IrqsStatus.value() & InterruptMask::IFSynchronization) != 0) {
            // I/Q IF Synchronization Failure handling
        }
        if ((irqStatus.rf24IrqsStatus.value() & InterruptMask::TransceiverError) != 0) {
            // Transceiver Error handling
        }
        if ((irqStatus.rf24IrqsStatus.value() & InterruptMask::BatteryLow) != 0) {
            // Battery Low handling
        }
        if ((irqStatus.rf24IrqsStatus.value() & InterruptMask::EnergyDetectionCompletion) != 0) {
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::ENERGY_DETECTION_24_COMPLETE));
        }
        if ((irqStatus.rf24IrqsStatus.value() & InterruptMask::TransceiverReady) != 0) {
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::TRANSCEIVER_24_READY));
        }
        if ((irqStatus.rf24IrqsStatus.value() & InterruptMask::Wakeup) != 0) {
            // Wakeup handling
        }

        //Baseband IRQ handling
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::FrameBufferLevelIndication) != 0) {
            // Frame Buffer Level Indication handling
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::AGCRelease) != 0) {
            // AGC Release handling
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::AGCHold) != 0) {
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::TransmitterFrameEnd) != 0) {
            // notify packetTransmissionBaseband() about successful transmission
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::BASEBAND_TX_24_COMPLETE));
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::ReceiverExtendMatch) != 0) {
            // Receiver Extended Match handling
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::ReceiverAddressMatch) != 0) {
            // Receiver Address Match handling
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::ReceiverFrameStart) != 0) {
            // reception started of frame started, do not allow the respective radio to be locked
            taskENTER_CRITICAL();
            basebandCoreIsReceiving24 = true;
            basebandCoreReceptionStartTime24 = xTaskGetTickCount();
            taskEXIT_CRITICAL();
        }
        if ((irqStatus.bbc1IrqsStatus.value() & InterruptMask::ReceiverFrameEnd) != 0) {
            if (auto status = getReceivedLength(Transceiver::RF24); !status.has_value()) {
                return etl::unexpected(status.error());
            } else {
                receivedPacketLength24 = status.value();
            }

            // Sanity check: Ensure received packet length fits in the destination buffer
            if (receivedPacketLength24 > destBuffer24.size()) {
                return etl::unexpected(Error::DESTINATION_BUFFER_TOO_SMALL);
            }

            if (auto status = spiBlockRead8(RegisterAddress::BBC1_FBRXS, destBuffer24.subspan(0, receivedPacketLength24)); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            // allow the respective radio to be locked again
            taskENTER_CRITICAL();
            basebandCoreIsReceiving24 = false;
            taskEXIT_CRITICAL();

            // notify waitForPacketReceptionBaseband()
            xEventGroupSetBits(eventGroupHandle, static_cast<uint32_t>(IrqEventGroupBit::BASEBAND_RX_24_COMPLETE));
        }

        return irqStatus;
    }

    /** =========== Private functions  =========== **/
    etl::expected<void, Error> AT86RF215Chip::waitForIrqEvent(
        MutexGuard& mutexGuard,
        IrqEventGroupBit irqEventGroupBit,
        uint16_t waitDelayMs) {

        const auto eventGroupBit = static_cast<uint32_t>(irqEventGroupBit);

        // clear possibly stale bit
        xEventGroupClearBits(eventGroupHandle, eventGroupBit);

        // release spi so that the interrupt handling task can read the irq registers
        mutexGuard.unlockSpi();

        if ((xEventGroupWaitBits(eventGroupHandle, eventGroupBit,
        pdTRUE, pdFALSE, pdMS_TO_TICKS(waitDelayMs)) & eventGroupBit) == false) {
            return etl::unexpected(Error::EVENT_WAIT_TIMEOUT);
        }

        if (!mutexGuard.lockSpi()) {
            return etl::unexpected(Error::MUTEX_LOCK_ERROR);
        }

        return {};
    }

    constexpr MorseCodeMapping AT86RF215Chip::getMorse(char c) {
        // . == 0 , - == 1 encoding starts from MSB
        switch (c) {
            // Letters (uppercase + lowercase)
            case 'A': case 'a': return { 0b01000000, 2 };  // .-
            case 'B': case 'b': return { 0b10000000, 4 };  // -...
            case 'C': case 'c': return { 0b10100000, 4 };  // -.-.
            case 'D': case 'd': return { 0b10000000, 3 };  // -..
            case 'E': case 'e': return { 0b00000000, 1 };  // .
            case 'F': case 'f': return { 0b00100000, 4 };  // ..-.
            case 'G': case 'g': return { 0b11000000, 3 };  // --.
            case 'H': case 'h': return { 0b00000000, 4 };  // ....
            case 'I': case 'i': return { 0b00000000, 2 };  // ..
            case 'J': case 'j': return { 0b01110000, 4 };  // .---
            case 'K': case 'k': return { 0b10100000, 3 };  // -.-
            case 'L': case 'l': return { 0b01000000, 4 };  // .-..
            case 'M': case 'm': return { 0b11000000, 2 };  // --
            case 'N': case 'n': return { 0b10000000, 2 };  // -.
            case 'O': case 'o': return { 0b11100000, 3 };  // ---
            case 'P': case 'p': return { 0b01100000, 4 };  // .--.
            case 'Q': case 'q': return { 0b11010000, 4 };  // --.-
            case 'R': case 'r': return { 0b01000000, 3 };  // .-.
            case 'S': case 's': return { 0b00000000, 3 };  // ...
            case 'T': case 't': return { 0b10000000, 1 };  // -
            case 'U': case 'u': return { 0b00100000, 3 };  // ..-
            case 'V': case 'v': return { 0b00010000, 4 };  // ...-
            case 'W': case 'w': return { 0b01100000, 3 };  // .--
            case 'X': case 'x': return { 0b10010000, 4 };  // -..-
            case 'Y': case 'y': return { 0b10110000, 4 };  // -.--
            case 'Z': case 'z': return { 0b11000000, 4 };  // --..

            // Digits
            case '0': return { 0b11111000, 5 }; // -----
            case '1': return { 0b01111000, 5 }; // .----
            case '2': return { 0b00111000, 5 }; // ..---
            case '3': return { 0b00011000, 5 }; // ...--
            case '4': return { 0b00001000, 5 }; // ....-
            case '5': return { 0b00000000, 5 }; // .....
            case '6': return { 0b10000000, 5 }; // -....
            case '7': return { 0b11000000, 5 }; // --...
            case '8': return { 0b11100000, 5 }; // ---..
            case '9': return { 0b11110000, 5 }; // ----.

            // Punctuation
            case '.': return { 0b01010100, 6 };  // .-.-.-
            case ',': return { 0b11001100, 6 };  // --..--
            case '?': return { 0b00110000, 6 };  // ..--..
            case '\'': return { 0b01111000, 6 };  // .----.
            case '!': return { 0b10101100, 6 };  // -.-.--
            case '/': return { 0b10010000, 5 };   // -..-.
            case '(': return { 0b10110000, 5 };   // -.--.
            case ')': return { 0b10110100, 6 };   // -.--.-
            case '&': return { 0b01000000, 5 };   // .-...
            case ':': return { 0b11100000, 6 };   // ---...
            case ';': return { 0b10101000, 6 };   // -.-.-.
            case '=': return { 0b10001000, 5 };   // -...-
            case '+': return { 0b01010000, 5 };   // .-.-.
            case '-': return { 0b10000100, 6 };   // -....-
            case '_': return { 0b00110100, 6 };   // ..--.-
            case '"': return { 0b01001000, 6 };   // .-..-.
            case '$': return { 0b00010010, 7 };   // ...-..-
            case '@': return { 0b01101000, 6 };   // .--.-.

            default:
                return { 0, 0 };  // not found
        }
    }

    etl::expected<void, Error> AT86RF215Chip::spiWrite8(RegisterAddress address, uint8_t value) {
        auto rawAddress = static_cast<uint16_t>(address);
        uint8_t msg[3] = {static_cast<uint8_t>(0x80 | ((rawAddress >> 8) & 0x7F)), static_cast<uint8_t>(rawAddress & 0xFF), value};

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET); // slave select pin

        if (HAL_SPI_Transmit(hspi, msg, 3, 3*SpiByteWriteCompleteDelayMs) != HAL_OK) {
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_WRITING_TO_REGISTER);
        }

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
        return {};
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::spiRead8(RegisterAddress address) {
        auto rawAddress = static_cast<uint16_t>(address);
        uint8_t msg[3] = {static_cast<uint8_t>((rawAddress >> 8) & 0x7F), static_cast<uint8_t>(rawAddress & 0xFF), 0x00};
        uint8_t response[3];

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET); // slave select pin

        if (HAL_SPI_TransmitReceive(hspi, msg, response, 3, 3*SpiByteReadCompleteDelayMs) != HAL_OK) {
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_READING_FROM_REGISTER);
        }

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
        return response[2];
    }

    etl::expected<void, Error> AT86RF215Chip::spiBlockWrite8(RegisterAddress address, etl::span<uint8_t> value) {
        auto rawAddress = static_cast<uint16_t>(address);
        uint8_t msg[3] = {static_cast<uint8_t>(0x80 | ((rawAddress >> 8) & 0x7F)), static_cast<uint8_t>(rawAddress & 0xFF)};

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET); // slave select pin

        if (HAL_SPI_Transmit(hspi, msg, 2, 2*SpiByteWriteCompleteDelayMs) != HAL_OK) {
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_WRITING_TO_REGISTER);
        }

        // clear possibly stale bit
        constexpr auto spiWriteCompleteGroupBit = static_cast<uint32_t>(IrqEventGroupBit::SPI_WRITE_COMPLETE);
        xEventGroupClearBits(eventGroupHandle, spiWriteCompleteGroupBit);

        if (HAL_SPI_Transmit_DMA(hspi, value.data(), value.size()) != HAL_OK) {
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_WRITING_TO_REGISTER);
        }

        EventBits_t eventBits = xEventGroupWaitBits(eventGroupHandle,
                            spiWriteCompleteGroupBit,
                            pdTRUE, pdTRUE,
                            pdMS_TO_TICKS(value.size() * static_cast<uint32_t>(SpiByteWriteCompleteDelayMs)));
        if (!(eventBits & spiWriteCompleteGroupBit)) {
            HAL_SPI_Abort(hspi);
            xEventGroupClearBits(eventGroupHandle, spiWriteCompleteGroupBit);
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_WRITING_TO_REGISTER);
        }

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::spiBlockRead8(RegisterAddress address, etl::span<uint8_t> response) {
        auto rawAddress = static_cast<uint16_t>(address);
        uint8_t msg[2] = {static_cast<uint8_t>((rawAddress >> 8) & 0x7F), static_cast<uint8_t>(rawAddress & 0xFF)};

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET);


        if (HAL_SPI_Transmit(hspi, msg, 2, 2 * SpiByteWriteCompleteDelayMs) != HAL_OK) {
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_READING_FROM_REGISTER);
        }

        // clear possibly stale bit
        constexpr auto spiWriteCompleteGroupBit = static_cast<uint32_t>(IrqEventGroupBit::SPI_WRITE_COMPLETE);
        constexpr auto spiReadCompleteGroupBit = static_cast<uint32_t>(IrqEventGroupBit::SPI_READ_COMPLETE);
        xEventGroupClearBits(eventGroupHandle, spiWriteCompleteGroupBit | spiReadCompleteGroupBit);

        if (HAL_SPI_Receive_DMA(hspi, response.data(), response.size()) != HAL_OK) {
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_READING_FROM_REGISTER);
        }

        EventBits_t eventBits = xEventGroupWaitBits(eventGroupHandle,
                            spiReadCompleteGroupBit,
                            pdTRUE, pdTRUE,
                            pdMS_TO_TICKS(response.size() * static_cast<uint32_t>(SpiByteReadCompleteDelayMs)));

        if (!(eventBits & spiReadCompleteGroupBit)) {
            HAL_SPI_Abort(hspi);
            xEventGroupClearBits(eventGroupHandle, spiWriteCompleteGroupBit | spiReadCompleteGroupBit);
            HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_READING_FROM_REGISTER);
        }

        HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_SET);
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::spiApplyBitwiseOr(RegisterAddress address, uint8_t mask) {
        if (auto regVal = spiRead8(address); !regVal.has_value()) {
            return etl::unexpected(regVal.error());
        } else {
            if (auto status = spiWrite8(address, regVal.value() | mask); !status.has_value()) {
                return status;
            }
        }
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::spiApplyBitwiseAnd(RegisterAddress address, uint8_t mask) {
        if (auto regVal = spiRead8(address); !regVal.has_value()) {
            return etl::unexpected(regVal.error());
        } else {
            if (auto status = spiWrite8(address, regVal.value() & mask); !status.has_value()) {
                return status;
            }
        }
        return {};
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::spiOverwriteBits(
        RegisterAddress address,
        uint8_t mask,
        uint8_t overwriteBits) {
        if (auto regVal = spiRead8(address); !regVal.has_value()) {
            return etl::unexpected(regVal.error());
        } else {
            if (auto status = spiWrite8(address, (regVal.value() & ~mask) | (overwriteBits & mask)); !status.has_value()) {
                return etl::unexpected(status.error());
            }
            return regVal.value();
        }
    }

    etl::expected<State, Error> AT86RF215Chip::getStatePrivate(Transceiver transceiver) {
        uint8_t state;
        RegisterAddress stateReg = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_STATE : RegisterAddress::RF24_STATE;
        if (auto status = spiRead8(stateReg); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            state = status.value() & 0x07;
        }

        if ((state < 0x02) || (state > 0x07)) {
            return etl::unexpected(Error::UKNOWN_REQUESTED_STATE);
        }

        return static_cast<State>(state);
    }

    etl::expected<void, Error> AT86RF215Chip::setStatePrivate(Transceiver transceiver, State stateCmd) {
        State currentState;
        if (auto status = getStatePrivate(transceiver); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            currentState = status.value();
        }

        if (currentState == stateCmd) {
            return {};
        }

        // check if transition to requested command is possible
        switch (stateCmd) {
            case State::RF_TRXOFF:
                break;
            case State::RF_TXPREP:
                if ((currentState != State::RF_TRXOFF) && (currentState != State::RF_RX) && (currentState != State::RF_TX)) {
                    return etl::unexpected(Error::FAILED_CHANGING_STATE);
                }
                break;
            case State::RF_TX:
                [[fallthrough]];
            case State::RF_RX:
                if (currentState != State::RF_TXPREP) {
                    return etl::unexpected(Error::FAILED_CHANGING_STATE);
                }
                break;
            case State::RF_NOP:
                [[fallthrough]];
            case State::RF_RESET:
                break;
            case State::RF_SLEEP:
                if ((currentState != State::RF_TRXOFF) && (currentState != State::RF_SLEEP)) {
                    return etl::unexpected(Error::FAILED_CHANGING_STATE);
                }
                break;
            default:
                return etl::unexpected(Error::UKNOWN_REQUESTED_STATE);
        }

        RegisterAddress cmdReg = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_CMD : RegisterAddress::RF24_CMD;
        if (auto status = spiWrite8(cmdReg, static_cast<uint8_t>(stateCmd)); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::setPllChannelSpacing(Transceiver transceiver, uint8_t spacing) {
        RegisterAddress regscs = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_CS : RegisterAddress::RF24_CS;
        return spiWrite8(regscs, spacing);
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::getPllChannelSpacing(Transceiver transceiver) {
        RegisterAddress regscs = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_CS : RegisterAddress::RF24_CS;
        return spiRead8(regscs);
    }

    etl::expected<void, Error> AT86RF215Chip::setPllChannelFrequency(Transceiver transceiver, uint16_t freq) {
        RegisterAddress regcf0h;
        RegisterAddress regcf0l;

        if (transceiver == Transceiver::RF09) {
            regcf0h = RegisterAddress::RF09_CCF0H;
            regcf0l = RegisterAddress::RF09_CCF0L;
        } else { // transceiver == RF24
            regcf0h = RegisterAddress::RF24_CCF0H;
            regcf0l = RegisterAddress::RF24_CCF0L;
        }

        if (auto status = spiWrite8(regcf0l, freq & 0x00FF); !status.has_value()) {
            return status;
        }
        return spiWrite8(regcf0h, (freq & 0xFF00) >> 8);
    }

    etl::expected<uint16_t, Error> AT86RF215Chip::getPllChannelFrequency(Transceiver transceiver) {
        RegisterAddress regcf0h;
        RegisterAddress regcf0l;

        if (transceiver == Transceiver::RF09) {
            regcf0h = RegisterAddress::RF09_CCF0H;
            regcf0l = RegisterAddress::RF09_CCF0L;
        } else { // transceiver == RF24
            regcf0h = RegisterAddress::RF24_CCF0H;
            regcf0l = RegisterAddress::RF24_CCF0L;
        }

        uint16_t cf0h;
        if (auto status = spiRead8(regcf0h); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            cf0h = status.value() & 0xFF;
        }

        uint16_t cf0l;
        if (auto status = spiRead8(regcf0l); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            cf0l = status.value() & 0xFF;
        }

        return (cf0h << 8) | cf0l;
    }

    etl::expected<uint16_t, Error> AT86RF215Chip::getPllChannelNumber(Transceiver transceiver) {
        RegisterAddress regcnl;
        RegisterAddress regcnm;

        if (transceiver == Transceiver::RF09) {
            regcnl = RegisterAddress::RF09_CNL;
            regcnm = RegisterAddress::RF09_CNM;
        } else { // transceiver == RF24
            regcnl = RegisterAddress::RF24_CNL;
            regcnm = RegisterAddress::RF24_CNM;
        }

        uint16_t cnl;
        if (auto status = spiRead8(regcnl); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            cnl = status.value();
        }

        uint16_t cnm;
        if (auto status = spiRead8(regcnm); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            cnm = status.value() & 0x01;
        }

        return (cnm << 8) | cnl;
    }

    etl::expected<void, Error> AT86RF215Chip::setPllBw(PLLBandwidth bw) {
        if (auto status = spiOverwriteBits(RegisterAddress::RF09_PLL, 0x30, static_cast<uint8_t>(bw) << 4); !status.has_value()) {
            return etl::unexpected(status.error());
        }
        return {};
    }

    etl::expected<PLLBandwidth, Error> AT86RF215Chip::getPllBw() {
        if (auto status = spiRead8(RegisterAddress::RF09_PLL); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t bw = (status.value() >> 4) & 0x03;
            if (bw > 0x03) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<PLLBandwidth>(bw);
        }
    }

    etl::expected<PLLState, Error> AT86RF215Chip::getPllState(Transceiver transceiver) {
        RegisterAddress regpll = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_PLL : RegisterAddress::RF24_PLL;
        uint8_t pllState;
        if (auto status = spiRead8(regpll); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            pllState = (status.value() >> 1) & 0x01;
        }
        return static_cast<PLLState>(pllState);
    }

    etl::expected<void, Error> AT86RF215Chip::configurePll(
        Transceiver transceiver,
        FrequencySynthesizerConfig& frequencySynthesizerConfig) {

        if (auto status = getStatePrivate(transceiver); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            if (status.value() != State::RF_TRXOFF) {
                return etl::unexpected(Error::INVALID_STATE_FOR_OPERATION);
            }
        }

        bool validConfigFlag;
        PLLChannelMode channelMode;
        uint64_t freq;
        PLLBandwidth bw;
        RegisterAddress ccf0h;
        RegisterAddress ccf0l;
        RegisterAddress cnm;
        RegisterAddress cs;
        RegisterAddress cnl;
        if (transceiver == Transceiver::RF09) {
            validConfigFlag = frequencySynthesizerConfig.validConfig09;
            channelMode = frequencySynthesizerConfig.channelMode09;
            freq = frequencySynthesizerConfig.frequency09;
            bw = frequencySynthesizerConfig.loopBandwidth09;
            ccf0h = RegisterAddress::RF09_CCF0H;
            ccf0l = RegisterAddress::RF09_CCF0L;
            cnm = RegisterAddress::RF09_CNM;
            cs = RegisterAddress::RF09_CS;
            cnl = RegisterAddress::RF09_CNL;
        } else {
            validConfigFlag = frequencySynthesizerConfig.validConfig24;
            channelMode = frequencySynthesizerConfig.channelMode24;
            freq = frequencySynthesizerConfig.frequency24;
            bw = frequencySynthesizerConfig.loopBandwidth24;
            ccf0h = RegisterAddress::RF24_CCF0H;
            ccf0l = RegisterAddress::RF24_CCF0L;
            cnm = RegisterAddress::RF24_CNM;
            cs = RegisterAddress::RF24_CS;
            cnl = RegisterAddress::RF24_CNL;
        }

        if (!validConfigFlag) {
            return etl::unexpected(Error::INVALID_TRANSCEIVER_FREQ);
        }

        if (channelMode == PLLChannelMode::IEECompliant) {
            // RFn_CCF0H, RFn_CCFOL:  high and low byte of central frequency
            // CNM.CHN, RFn_CNL: high bit and low byte of channel number
            // RFn_CS: channel spacing
            // @TODO: central frequency and channel spacing for each band in 68d, 68e tables of IEEE Std 802.15.4g™-2012
            return etl::unexpected(Error::INVALID_TRANSCEIVER_FREQ);
        } else {
            // RFn_CCF0H, RFn_CCF0L, RFn_CNL: high, middle and low byte of N_channnel
            uint64_t Nchannel;
            if (channelMode == PLLChannelMode::FineResolution450) {
                Nchannel = (freq - 377000) * 65536 / 6500;
            }
            else if (channelMode == PLLChannelMode::FineResolution900) {
                Nchannel = (freq - 754000) * 65536 / 13000;
            }
            else {
                Nchannel = (freq - 2366000) * 65536 / 26000;
            }

            if (auto status = spiWrite8(ccf0h, static_cast<uint8_t>(Nchannel >> 16)); !status.has_value()) {
                return status;
            }

            if (auto status = spiWrite8(ccf0l, static_cast<uint8_t>(Nchannel >> 8)); !status.has_value()) {
                return status;
            }

            if (auto status = spiWrite8(cnl, static_cast<uint8_t>(Nchannel)); !status.has_value()) {
                return status;
            }
        }

        // Configure channel mode. According to p. 6.3.2, the the RFn_CNM register must always be written last
        if (auto status = spiOverwriteBits(cnm, 0xC0, static_cast<uint8_t>(channelMode) << 6); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // RFn_PLL
        return setPllBw(bw);
    }

    etl::expected<DevicePartNumber, Error> AT86RF215Chip::getPartNumber() {
        uint8_t dpn;
        if (auto status = spiRead8(RegisterAddress::RF_PN); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            dpn = status.value();
        }

        if ((dpn < 0x34) || (dpn > 0x36)) {
            return etl::unexpected(Error::INVALID_REGISTER_VALUE);
        }

        return static_cast<DevicePartNumber>(dpn);
    }

    etl::expected<DeviceVersionNumber, Error> AT86RF215Chip::getVersionNumber() {
        uint8_t vn;
        if (auto status = spiRead8(RegisterAddress::RF_VN); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            vn = status.value();
        }

        if ((vn != 0x01) && (vn != 0x03)) {
            return etl::unexpected(Error::INVALID_REGISTER_VALUE);
        }
        return static_cast<DeviceVersionNumber>(vn);
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::getPllFrequency(Transceiver transceiver) {
        RegisterAddress regpll = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_PLLCF : RegisterAddress::RF24_PLLCF;

        if (auto status = spiRead8(regpll); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return status.value() & 0x3F;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setTcxoTrimming(CrystalTrim trim) {
        if (auto status = spiOverwriteBits(RegisterAddress::RF_XOC, 0x0F, static_cast<uint8_t>(trim)); !status.has_value()) {
            return etl::unexpected(status.error());
        }
        return {};
    }

    etl::expected<CrystalTrim, Error> AT86RF215Chip::readTcxoTrimming() {
        if (auto status = spiRead8(RegisterAddress::RF_XOC); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return static_cast<CrystalTrim>(status.value() & 0x0F);
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setTcxoFastStartUpEnable(bool fastStartUp) {
        if (auto status = spiOverwriteBits(RegisterAddress::RF_XOC, 0x10, static_cast<uint8_t>(fastStartUp) << 4); !status.has_value()) {
            return etl::unexpected(status.error());
        }
        return {};
    }

    etl::expected<bool, Error> AT86RF215Chip::readTcxoFastStartUpEnable() {
        if (auto status = spiRead8(RegisterAddress::RF_XOC); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return (status.value() & 0x10) >> 4;
        }
    }

    etl::expected<PowerAmplifierRampTime, Error> AT86RF215Chip::getPaRampUpTime(Transceiver transceiver) {
        RegisterAddress regtxcutc = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_TXCUTC : RegisterAddress::RF24_TXCUTC;

        if (auto status = spiRead8(regtxcutc); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t paRampUp = (status.value() & 0xC0) >> 6;
            if (paRampUp > 0x03) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<PowerAmplifierRampTime>(paRampUp);
        }
    }

    etl::expected<TransmitterCutOffFrequency, Error> AT86RF215Chip::getCutoffFreq(Transceiver transceiver) {
        RegisterAddress regtxcutc = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_TXCUTC : RegisterAddress::RF24_TXCUTC;

        if (auto status = spiRead8(regtxcutc); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return static_cast<TransmitterCutOffFrequency>(status.value() & 0x0F);
        }
    }


    etl::expected<TxRelativeCutoffFrequency, Error> AT86RF215Chip::getRelativeCutoffFreq(Transceiver transceiver) {
        RegisterAddress regtxdfe = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_TXDFE : RegisterAddress::RF24_TXDFE;

        if (auto status = spiRead8(regtxdfe); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t txRelCutoff = (status.value() & 0xE0) >> 5;
            if (txRelCutoff > 0x04) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<TxRelativeCutoffFrequency>(txRelCutoff);
        }
    }


    etl::expected<bool, Error> AT86RF215Chip::getDirectModulation(Transceiver transceiver) {
        RegisterAddress regtxdfe = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_TXDFE : RegisterAddress::RF24_TXDFE;

        if (auto status = spiRead8(regtxdfe); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return (status.value() & 0x10) >> 4;
        }
    }


    etl::expected<ReceiverSampleRate, Error> AT86RF215Chip::getSampleRate(Transceiver transceiver) {
        RegisterAddress regtxdfe = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_RXDFE : RegisterAddress::RF24_RXDFE;

        if (auto status = spiRead8(regtxdfe); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t sampleRate = status.value() & 0x0F;
            if (sampleRate == 0 || (sampleRate > 0x06 && sampleRate != 0x08 && sampleRate != 0x0A)) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<ReceiverSampleRate>(sampleRate);
        }
    }

    etl::expected<PowerAmplifierCurrentControl, Error> AT86RF215Chip::getPaDcCurrent(Transceiver transceiver) {
        RegisterAddress regpac = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_PAC : RegisterAddress::RF24_PAC;

        if (auto status = spiRead8(regpac); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t pac = (status.value() & 0x60) >> 5;
            if (pac > 0x03) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<PowerAmplifierCurrentControl>(pac);
        }
    }


    etl::expected<bool, Error> AT86RF215Chip::getLnaBypassed(Transceiver transceiver) {
        RegisterAddress regaux = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;

        if (auto status = spiRead8(regaux); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return status.value() & 0x80;
        }
    }

    etl::expected<AutomaticGainControlMAP, Error> AT86RF215Chip::getAgcmap(Transceiver transceiver) {
        RegisterAddress regaux = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;

        if (auto status = spiRead8(regaux); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t agcmap = (status.value() & 0x60) >> 5;
            if (agcmap > 0x03) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<AutomaticGainControlMAP>(agcmap);
        }
    }


    etl::expected<AutomaticVoltageExternal, Error> AT86RF215Chip::getExternalAnalogVoltage(
        Transceiver transceiver) {
        RegisterAddress regaux = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;

        if (auto status = spiRead8(regaux); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t avext = (status.value() & 0x10) >> 4;
            if (avext > 0x02) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<AutomaticVoltageExternal>(avext);
        }
    }

    etl::expected<bool, Error> AT86RF215Chip::getAnalogVoltageSettledStatus(Transceiver transceiver) {
        RegisterAddress regaux = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;

        if (auto status = spiRead8(regaux); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return (status.value() & 0x04) >> 2;
        }
    }

    etl::expected<PowerAmplifierVoltageControl, Error> AT86RF215Chip::getAnalogPowerAmplifierVoltage(
            Transceiver transceiver) {
        RegisterAddress regaux = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;

        if (auto status = spiRead8(regaux); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t pavc = status.value() & 0x03;
            if (pavc == 0x03) {
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
            return static_cast<PowerAmplifierVoltageControl>(pavc);
        }
    }


    etl::expected<void, Error> AT86RF215Chip::setEdAverageDetection(
        Transceiver transceiver,
        uint8_t df,
        EnergyDetectionTimeBasis dtb) {
        RegisterAddress regedd = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_EDD : RegisterAddress::RF24_EDD;

        uint8_t reg = ((df & 0x3F) << 2) | (static_cast<uint8_t>(dtb) & 0x3);
        return spiWrite8(regedd, reg);
    }

    etl::expected<uint16_t, Error> AT86RF215Chip::getEdAverageDetection(Transceiver transceiver) {
        RegisterAddress regedd = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_EDD : RegisterAddress::RF24_EDD;

        if (auto status = spiRead8(regedd); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            uint8_t df = (status.value() & 0xFC) >> 2;
            uint8_t dtb = (status.value() & 0x3);

            switch (dtb) {
            case 0x0:
                return df * 2;
            case 0x1:
                return df * 8;
            case 0x2:
                return df * 32;
            case 0x3:
                return df * 128;
            default:
                return etl::unexpected(Error::INVALID_REGISTER_VALUE);
            }
        }
    }

    etl::expected<int8_t, Error> AT86RF215Chip::getReceiverEnergyDetection(Transceiver transceiver) {
        RegisterAddress regedv = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_EDV : RegisterAddress::RF24_EDV;

        if (auto status = spiRead8(regedv); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            int8_t energy = static_cast<int8_t>(status.value());
            if (energy > 4) {
                return etl::unexpected(Error::INVALID_RSSI_MEASUREMENT);
            }
            return energy;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setBatteryMonitorControl(
        BatteryMonitorHighRange range,
        BatteryMonitorVoltageThreshold threshold) {

        if (auto status = setBatteryMonitorHighRange(range); !status.has_value()) {
            return status;
        }

        if (auto status = setBatteryMonitorVoltageThreshold(threshold); !status.has_value()) {
            return status;
        }

        return {};
    }


    etl::expected<BatteryMonitorStatus, Error> AT86RF215Chip::getBatteryMonitorStatus() {
        if (auto status = spiRead8(RegisterAddress::RF_BMDVC); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return static_cast<BatteryMonitorStatus>((status.value() & 0x20) >> 5);
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setBatteryMonitorHighRange(BatteryMonitorHighRange range) {
        if (auto status = spiOverwriteBits(RegisterAddress::RF_BMDVC, 0x10, static_cast<uint8_t>(range) << 4); !status.has_value()) {
            return etl::unexpected(status.error());
        }
        return {};
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::getBatteryMonitorHighRange() {
        if (auto status = spiRead8(RegisterAddress::RF_BMDVC); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return (status.value() & 0x10) >> 4;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setBatteryMonitorVoltageThreshold(BatteryMonitorVoltageThreshold threshold) {
        if (auto status = spiOverwriteBits(RegisterAddress::RF_BMDVC, 0x0F, static_cast<uint8_t>(threshold)); !status.has_value()) {
            return etl::unexpected(status.error());
        }
        return {};
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::getBatteryMonitorVoltageThreshold() {
        if (auto status = spiRead8(RegisterAddress::RF_BMDVC); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return status.value() & 0x0F;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setExternalFrontEndControl(Transceiver transceiver,
        ExternalFrontEndControl frontEndControl) {
        RegisterAddress regAddress = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_PADFE : RegisterAddress::RF24_PADFE;

        if (auto status = spiOverwriteBits(regAddress, 0xC0, static_cast<uint8_t>(frontEndControl) << 6); !status.has_value()) {
            return etl::unexpected(status.error());
        }
        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::setupTxFrontend(Transceiver transceiver,
        PowerAmplifierRampTime paRampTime,
        TransmitterCutOffFrequency cutoff,
        TxRelativeCutoffFrequency txRelCutoff,
        DirectModEnableFSKDM directMod,
        TransmitterSampleRate txSampleRate,
        PowerAmplifierCurrentControl paCurrControl,
        uint8_t txOutPower,
        ExternalLNABypass extLnaBypass,
        AutomaticGainControlMAP agcMap,
        AutomaticVoltageExternal avgExt,
        AnalogVoltageEnable avEnable,
        PowerAmplifierVoltageControl paVcontrol,
        ExternalFrontEndControl externalFrontEndControl) {
        RegisterAddress regtxcut;
        RegisterAddress regtxdfe;
        RegisterAddress regpac;
        RegisterAddress regauxs;

        uint8_t reg = 0;

        if (transceiver == Transceiver::RF09) {
            regtxcut = RegisterAddress::RF09_TXCUTC;
            regtxdfe = RegisterAddress::RF09_TXDFE;
            regpac = RegisterAddress::RF09_PAC;
            regauxs = RegisterAddress::RF09_AUXS;
        } else {
            regtxcut = RegisterAddress::RF24_TXCUTC;
            regtxdfe = RegisterAddress::RF24_TXDFE;
            regpac = RegisterAddress::RF24_PAC;
            regauxs = RegisterAddress::RF24_AUXS;
        }

        // Set RFn_TXCUTC
        reg = (static_cast<uint8_t>(paRampTime) << 6) | static_cast<uint8_t>(cutoff);
        if (auto status = spiWrite8(regtxcut, reg); !status.has_value()) {
            return status;
        }

        // Set RFn_TXDFE
        reg = (static_cast<uint8_t>(txRelCutoff) << 5) | static_cast<uint8_t>(directMod) << 4 | static_cast<uint8_t>(txSampleRate);
        if (auto status = spiWrite8(regtxdfe, reg); !status.has_value()) {
            return status;
        }

        // Set RFn_PAC
        reg = (static_cast<uint8_t>(paCurrControl) << 5) | (txOutPower & 0x1F);
        if (auto status = spiWrite8(regpac, reg); !status.has_value()) {
            return status;
        }

        // Set RFn_AUXS
        reg = (static_cast<uint8_t>(extLnaBypass) << 7) | (static_cast<uint8_t>(agcMap) << 5) | (static_cast<uint8_t>(avgExt) << 4) | (static_cast<uint8_t>(avEnable) << 3) | (static_cast<uint8_t>(paVcontrol));
        if (auto status = spiWrite8(regauxs, reg); !status.has_value()) {
            return status;
        }

        // Set RFn_PADFE
        return setExternalFrontEndControl(transceiver, externalFrontEndControl);
    }

    etl::expected<void, Error> AT86RF215Chip::setupIq(
        ExternalLoopback externalLoop,
        IQOutputCurrent outCur,
        IQmodeVoltage commonModeVol,
        IQmodeVoltageIEE commonModeIee,
        EmbeddedControlTX embeddedTxStart,
        ChipMode chipMode,
        SkewAlignment skewAlignment) {
        // Set RF_IQIFC0
        uint8_t reg;
        reg = (static_cast<uint8_t>(externalLoop) << 7) | (static_cast<uint8_t>(outCur) << 4) | (static_cast<uint8_t>(commonModeVol) << 2) | (static_cast<uint8_t>(commonModeIee) << 1) | static_cast<uint8_t>(embeddedTxStart);
        if (auto status = spiWrite8(RegisterAddress::RF_IQIFC0, reg); !status.has_value()) {
            return status;
        }

        // Set RF_IQIFC1
        reg = (static_cast<uint8_t>(chipMode) << 4) | static_cast<uint8_t>(skewAlignment);
        return spiWrite8(RegisterAddress::RF_IQIFC1, reg);
    }

    etl::expected<bool, Error> AT86RF215Chip::getIqSyncStatus() {
        if (auto status = spiRead8(RegisterAddress::RF_IQIFC2); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return status.value() >> 7;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setupCrystal(bool fast_start_up, CrystalTrim crystal_trim) {
       if (auto status = setTcxoFastStartUpEnable(fast_start_up); !status.has_value()) {
           return status;
       }

       return setTcxoTrimming(crystal_trim);
    }

    etl::expected<void, Error> AT86RF215Chip::setupRxEnergyDetection(
        Transceiver transceiver,
        EnergyDetectionMode energyMode,
        uint8_t energyDetectFactor,
        EnergyDetectionTimeBasis energyTimeBasis) {
        uint8_t regValue;
        RegisterAddress regedc;
        RegisterAddress regedd;

        if (transceiver == Transceiver::RF09) {
            regedc = RegisterAddress::RF09_EDC;
            regedd = RegisterAddress::RF09_EDD;
        } else if (transceiver == Transceiver::RF24) {
            regedc = RegisterAddress::RF24_EDC;
            regedd = RegisterAddress::RF24_EDD;
        }

        // Set RFn_EDC
        if (auto status = spiOverwriteBits(regedc, 0x03, static_cast<uint8_t>(energyMode)); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        // Set RFn_EDD
        regValue = (energyDetectFactor << 2) | static_cast<uint8_t>(energyTimeBasis);
        return spiWrite8(regedd, regValue);
    }

    etl::expected<void, Error> AT86RF215Chip::setupRxFrontend(
        Transceiver transceiver,
        bool ifInversion,
        bool ifShift,
        ReceiverBandwidth rxBw,
        RxRelativeCutoffFrequency rxRelCutoff,
        ReceiverSampleRate rxSampleRate,
        bool agcInput,
        AverageTimeNumberSamples agcAvgSample,
        AGCReset agcReset,
        AGCFreezeControl agcFreezeControl,
        AGCEnable agcEnable,
        AutomaticGainTarget agcTarget,
        uint8_t gainControlWord) {
        if (gainControlWord > 23) {
            return etl::unexpected(Error::INVALID_AGC_CONTROl_WORD);
        }

        RegisterAddress regrxbwc;
        RegisterAddress regrxdfe;
        RegisterAddress regagcc;
        RegisterAddress regagcs;

        if (transceiver == Transceiver::RF09) {
            regrxbwc = RegisterAddress::RF09_RXBWC;
            regrxdfe = RegisterAddress::RF09_RXDFE;
            regagcc = RegisterAddress::RF09_AGCC;
            regagcs = RegisterAddress::RF09_AGCS;
        } else {
            regrxbwc = RegisterAddress::RF24_RXBWC;
            regrxdfe = RegisterAddress::RF24_RXDFE;
            regagcc = RegisterAddress::RF24_AGCC;
            regagcs = RegisterAddress::RF24_AGCS;
        }

        /// Set RFn_RXBWC
        uint8_t rxbwcMask = 0x3F;
        uint8_t rxbwcVal = (static_cast<uint8_t>(ifInversion) << 5) | (static_cast<uint8_t>(ifShift) << 4) | static_cast<uint8_t>(rxBw);
        if (auto status = spiOverwriteBits(regrxbwc, rxbwcMask, rxbwcVal); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        /// Set RFn_RXDFE
        uint8_t rxdfeMask = 0xEF;
        uint8_t rxdfeVal = (static_cast<uint8_t>(rxRelCutoff) << 5) | static_cast<uint8_t>(rxSampleRate);
        if (auto status = spiOverwriteBits(regrxdfe, rxdfeMask, rxdfeVal); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        /// Set RFn_AGGC
        uint8_t agccMask = 0x7F;
        uint8_t agccVal = (static_cast<uint8_t>(agcInput) << 6) | (static_cast<uint8_t>(agcAvgSample) << 4) | (static_cast<uint8_t>(agcReset) << 3) | (static_cast<uint8_t>(agcFreezeControl) << 1) | static_cast<uint8_t>(agcEnable);
        if (auto status = spiOverwriteBits(regagcc, agccMask, agccVal); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        /// Set RFn_AGCS
        return spiWrite8(regagcs, (static_cast<uint8_t>(agcTarget) << 5) | gainControlWord);
    }

    etl::expected<void, Error> AT86RF215Chip::setupIrqCfg(
        bool maskMode,
        IRQPolarity polarity,
        PadDriverStrength padDriverStrength) {
        uint8_t mask = 0x0F;
        uint8_t val = (maskMode << 3) | (static_cast<uint8_t>(polarity) << 2) | static_cast<uint8_t>(padDriverStrength);
        if (auto status = spiOverwriteBits(RegisterAddress::RF_CFG, mask, val); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::setupPhyBaseband(
        Transceiver transceiver,
        bool continuousTransmit,
        bool frameSeqFilter,
        bool transmitterAutoFCS,
        FrameCheckSequenceType fcsType,
        bool basebandEnable,
        PhysicalLayerType phyType) {
        RegisterAddress regphy = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_PC : RegisterAddress::BBC1_PC;

        return spiWrite8(regphy,
            (continuousTransmit << 7) |
            (frameSeqFilter << 6) |
            (transmitterAutoFCS << 4) |
            (static_cast<uint8_t>(fcsType) << 3) |
            (basebandEnable << 2) |
            static_cast<uint8_t>(phyType));
    }

    etl::expected<void, Error> AT86RF215Chip::setupIrqMask(
        Transceiver transceiver,
        bool iqIfSynchronizationFailure,
        bool transceiverError,
        bool batteryLow,
        bool energyDetectionCompletion,
        bool transceiverReady,
        bool wakeup,
        bool frameBufferLevelIndication,
        bool agcRelease,
        bool agcHold,
        bool transmitterFrameEnd,
        bool receiverExtendedMatch,
        bool receiverAddressMatch,
        bool receiverFrameEnd,
        bool receiverFrameStart) {
        RegisterAddress regbbc;
        RegisterAddress regrf;

        if (transceiver == Transceiver::RF09) {
            regbbc = RegisterAddress::BBC0_IRQM;
            regrf = RegisterAddress::RF09_IRQM;
        } else if (transceiver == Transceiver::RF24) {
            regbbc = RegisterAddress::BBC1_IRQM;
            regrf = RegisterAddress::RF24_IRQM;
        }

        if (auto status = spiWrite8(regrf,
            iqIfSynchronizationFailure << 5 |
            transceiverError << 4 |
            batteryLow << 3 |
            energyDetectionCompletion << 2 |
            transceiverReady << 1 |
            wakeup); !status.has_value()) {
            return status;
        }

        if (auto status = spiWrite8(regbbc,
        frameBufferLevelIndication << 7 |
            agcRelease << 6 |
            agcHold << 5 |
            transmitterFrameEnd << 4 |
            receiverExtendedMatch << 3 |
            receiverAddressMatch << 2 |
            receiverFrameEnd << 1 |
            receiverFrameStart); !status.has_value()) {
            return status;
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::setup() {
        // Check state of RF09 core
        if (auto state = getStatePrivate(Transceiver::RF09); !state.has_value()) {
            return etl::unexpected(state.error());
        } else {
            // We have access to all registers only if we are in the state TRXOFF
            if (state.value() != State::RF_TRXOFF) {
                return etl::unexpected(Error::INVALID_STATE_FOR_OPERATION);
            }
        }

        // Check state of RF24 core
        if (auto state = getStatePrivate(Transceiver::RF24); !state.has_value()) {
            return etl::unexpected(state.error());
        } else {
            // We have access to all registers only if we are in the state TRXOFF
            if (state.value() != State::RF_TRXOFF) {
                return etl::unexpected(Error::INVALID_STATE_FOR_OPERATION);
            }
        }

        // Set IRQ masks
        if (auto status = setupIrqMask(Transceiver::RF09, radioInterruptsConfig.iqIfSynchronizationFailure09, radioInterruptsConfig.transceiverError09,
                       radioInterruptsConfig.batteryLow09, radioInterruptsConfig.energyDetectionCompletion09, radioInterruptsConfig.transceiverReady09,
                       radioInterruptsConfig.wakeup09, basebandCoreInterruptsConfig.frameBufferLevelIndication09, basebandCoreInterruptsConfig.agcRelease09,
                       basebandCoreInterruptsConfig.agcHold09, basebandCoreInterruptsConfig.transmitterFrameEnd09, basebandCoreInterruptsConfig.receiverExtendedMatch09,
                       basebandCoreInterruptsConfig.receiverAddressMatch09, basebandCoreInterruptsConfig.receiverFrameEnd09, basebandCoreInterruptsConfig.receiverFrameStart09);
            !status.has_value()) {
            return status;
        }

        if (auto status = setupIrqMask(Transceiver::RF24, radioInterruptsConfig.iqIfSynchronizationFailure24, radioInterruptsConfig.transceiverError24,
                       radioInterruptsConfig.batteryLow24, radioInterruptsConfig.energyDetectionCompletion24, radioInterruptsConfig.transceiverReady24,
                       radioInterruptsConfig.wakeup24, basebandCoreInterruptsConfig.frameBufferLevelIndication24, basebandCoreInterruptsConfig.agcRelease24,
                       basebandCoreInterruptsConfig.agcHold24, basebandCoreInterruptsConfig.transmitterFrameEnd24, basebandCoreInterruptsConfig.receiverExtendedMatch24,
                       basebandCoreInterruptsConfig.receiverAddressMatch24, basebandCoreInterruptsConfig.receiverFrameEnd24, basebandCoreInterruptsConfig.receiverFrameStart24);
            !status.has_value()) {
            return status;
        }

        // Set IRQ pin
        if (auto status = setupIrqCfg(generalConfig.irqMaskMode, generalConfig.irqPolarity,
                      generalConfig.padDriverStrength); !status.has_value()) {
            return status;
        }

        // Set PLL
        if (auto status = configurePll(Transceiver::RF09, freqSynthesizerConfig); !status.has_value()) {
            return status;
        }

        if (auto status = configurePll(Transceiver::RF24, freqSynthesizerConfig); !status.has_value()) {
            return status;
        }

        // Setup Physical Layer for Baseband Cores
        if (auto status = setupPhyBaseband(Transceiver::RF09, basebandCoreConfig.continuousTransmit09,
                           basebandCoreConfig.frameCheckSequenceFilterEn09, basebandCoreConfig.transmitterAutoFrameCheckSequence09,
                           basebandCoreConfig.frameCheckSequenceType09, basebandCoreConfig.baseBandEnable09,
                           basebandCoreConfig.physicalLayerType09); !status.has_value()) {
            return status;
        }

        if (auto status = setupPhyBaseband(Transceiver::RF24, basebandCoreConfig.continuousTransmit24,
                           basebandCoreConfig.frameCheckSequenceFilterEn24, basebandCoreConfig.transmitterAutoFrameCheckSequence24,
                           basebandCoreConfig.frameCheckSequenceType24, basebandCoreConfig.baseBandEnable24,
                           basebandCoreConfig.physicalLayerType24); !status.has_value()) {
            return status;
        }

        /// BBCn_FSKC0
        if (auto status = setBbcFskc0Config(Transceiver::RF09, basebandCoreConfig.bandwidth_time_09, basebandCoreConfig.midxs_09, basebandCoreConfig.midx_09, basebandCoreConfig.mord_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskc0Config(Transceiver::RF24, basebandCoreConfig.bandwidth_time_24, basebandCoreConfig.midxs_24, basebandCoreConfig.midx_24, basebandCoreConfig.mord_24); !status.has_value()) {
            return status;
        }

        /// BBCn_FSKC1
        if (auto status = setBbcFskc1Config(Transceiver::RF09, basebandCoreConfig.freq_inv_09, basebandCoreConfig.sr_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskc1Config(Transceiver::RF24, basebandCoreConfig.freq_inv_24, basebandCoreConfig.sr_24); !status.has_value()) {
            return status;
        }

        /// BBCn_FSKC2
        if (auto status = setBbcFskc2Config(Transceiver::RF09, basebandCoreConfig.preamble_detection_09, basebandCoreConfig.receiver_override_09, basebandCoreConfig.receiver_preamble_timeout_09, basebandCoreConfig.mode_switch_en_09, basebandCoreConfig.preamble_inversion_09, basebandCoreConfig.fec_scheme_09, basebandCoreConfig.interleaving_enable_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskc2Config(Transceiver::RF24, basebandCoreConfig.preamble_detection_24, basebandCoreConfig.receiver_override_24, basebandCoreConfig.receiver_preamble_timeout_24, basebandCoreConfig.mode_switch_en_24, basebandCoreConfig.preamble_inversion_24, basebandCoreConfig.fec_scheme_24, basebandCoreConfig.interleaving_enable_24); !status.has_value()) {
            return status;
        }

        /// BBCn_FSKC3
        if (auto status = setBbcFskc3Config(Transceiver::RF09, basebandCoreConfig.sfdt_09, basebandCoreConfig.prdt_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskc3Config(Transceiver::RF24, basebandCoreConfig.sfdt_24, basebandCoreConfig.prdt_24); !status.has_value()) {
            return status;
        }

        /// BBC_FSKC4
        if (auto status = setBbcFskc4Config(Transceiver::RF09, basebandCoreConfig.sfdQuantization_09, basebandCoreConfig.sfd32_09, basebandCoreConfig.rawModeReversalBit_09, basebandCoreConfig.csfd1_09, basebandCoreConfig.csfd0_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskc4Config(Transceiver::RF24, basebandCoreConfig.sfdQuantization_24, basebandCoreConfig.sfd32_24, basebandCoreConfig.rawModeReversalBit_24, basebandCoreConfig.csfd1_24, basebandCoreConfig.csfd0_24); !status.has_value()) {
            return status;
        }

        /// BBCn_FSKPHRTX
        if (auto status = setBbcFskphrtx(Transceiver::RF09, basebandCoreConfig.sfdUsed_09, basebandCoreConfig.dataWhitening_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskphrtx(Transceiver::RF24, basebandCoreConfig.sfdUsed_24, basebandCoreConfig.dataWhitening_24); !status.has_value()) {
            return status;
        }

        /// BBCn_FSKDM
        if (auto status = setBbcFskdm(Transceiver::RF09, basebandCoreConfig.fskPreamphasisEnable_09, basebandCoreConfig.directModEnableFskdm_09); !status.has_value()) {
            return status;
        }

        if (auto status = setBbcFskdm(Transceiver::RF24, basebandCoreConfig.fskPreamphasisEnable_24, basebandCoreConfig.directModEnableFskdm_24); !status.has_value()) {
            return status;
        }

        // Set TX front-end
        if (auto status = setupTxFrontend(Transceiver::RF09, txConfig.powerAmplifierRampTime09,
                          txConfig.transmitterCutOffFrequency09,
                          txConfig.txRelativeCutoffFrequency09, txConfig.directModulation09,
                          txConfig.transceiverSampleRate09,
                          txConfig.powerAmplifierCurrentControl09, txConfig.txOutPower09,
                          externalFrontEndConfig.externalLNABypass09, externalFrontEndConfig.automaticGainControlMAP09,
                          externalFrontEndConfig.automaticVoltageExternal09, externalFrontEndConfig.analogVoltageEnable09,
                          externalFrontEndConfig.powerAmplifierVoltageControl09, externalFrontEndConfig.externalFrontEnd_09); !status.has_value()) {
            return status;
        }

        if (auto status = setupTxFrontend(Transceiver::RF24, txConfig.powerAmplifierRampTime24,
                          txConfig.transmitterCutOffFrequency24,
                          txConfig.txRelativeCutoffFrequency24, txConfig.directModulation24,
                          txConfig.transceiverSampleRate24,
                          txConfig.powerAmplifierCurrentControl24, txConfig.txOutPower24,
                          externalFrontEndConfig.externalLNABypass24, externalFrontEndConfig.automaticGainControlMAP24,
                          externalFrontEndConfig.automaticVoltageExternal24, externalFrontEndConfig.analogVoltageEnable24,
                          externalFrontEndConfig.powerAmplifierVoltageControl24, externalFrontEndConfig.externalFrontEnd_24); !status.has_value()) {
            return status;
        }

        // Set up RX front-end
        if (auto status = setupRxFrontend(Transceiver::RF09, rxConfig.ifInversion09, rxConfig.ifShift09,
                          rxConfig.receiverBandwidth09, rxConfig.rxRelativeCutoffFrequency09,
                          rxConfig.receiverSampleRate09, rxConfig.agcInput09,
                          rxConfig.averageTimeNumberSamples09, rxConfig.agcReset_09, rxConfig.agcFreezeControl_09, rxConfig.agcEnabled09,
                          rxConfig.automaticGainTarget09, rxConfig.gainControlWord09); !status.has_value()) {
            return status;
        }

        if (auto status = setupRxFrontend(Transceiver::RF24, rxConfig.ifInversion24, rxConfig.ifShift24,
                          rxConfig.receiverBandwidth24, rxConfig.rxRelativeCutoffFrequency24,
                          rxConfig.receiverSampleRate24, rxConfig.agcInput24,
                          rxConfig.averageTimeNumberSamples24, rxConfig.agcReset_24, rxConfig.agcFreezeControl_24, rxConfig.agcEnabled24,
                          rxConfig.automaticGainTarget24, rxConfig.gainControlWord24); !status.has_value()) {
            return status;
        }

        // Set up IQ interface
        if (auto status = setupIq(iqInterfaceConfig.externalLoopback, iqInterfaceConfig.iqOutputCurrent,
                 iqInterfaceConfig.iqmodeVoltage, iqInterfaceConfig.iqmodeVoltageIEE,
                 iqInterfaceConfig.embeddedControlTX, iqInterfaceConfig.chipMode, iqInterfaceConfig.skewAlignment); !status.has_value()) {
            return status;
        }

        // Setup DAC Override registers
        if (auto status =
            setTxDaci(Transceiver::RF09, txConfig.enableInputI09, txConfig.dataInputI09); !status.has_value()) {
            return status;
        }

        if (auto status =
            setTxDaci(Transceiver::RF24, txConfig.enableInputI24, txConfig.dataInputI24); !status.has_value()) {
            return status;
        }

        if (auto status =
            setTxDacq(Transceiver::RF09, txConfig.enableInputQ09, txConfig.dataInputQ09); !status.has_value()) {
            return status;
        }

        if (auto status =
            setTxDacq(Transceiver::RF24, txConfig.enableInputQ24, txConfig.dataInputQ24); !status.has_value()) {
            return status;
        }

        /// Set up energy detection
        /// RFn_EDC, RFn_EDD
        if (auto status = setupRxEnergyDetection(
            Transceiver::RF09,
            rxConfig.energyDetectionMode09,
            rxConfig.energyDetectDurationFactor09,
            rxConfig.energyDetectionBasis09); !status.has_value()) {
            return status;
        }

        if (auto status = setupRxEnergyDetection(
            Transceiver::RF24,
            rxConfig.energyDetectionMode24,
            rxConfig.energyDetectDurationFactor24,
            rxConfig.energyDetectionBasis24); !status.has_value()) {
            return status;
        }

        /// Set up battery
        /// RF_BMDVC
        if (auto status = setBatteryMonitorControl(
            generalConfig.batteryMonitorHighRange,
            generalConfig.batteryMonitorVoltage); !status.has_value()) {
            return status;
        }

        /// Set up crystal oscillator
        /// RF_XOC
        return setupCrystal(generalConfig.fastStartUp, generalConfig.crystalTrim);
    }

    etl::expected<uint8_t, Error> AT86RF215Chip::getIrq(Transceiver transceiver) {
        RegisterAddress irqsReg = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_IRQS : RegisterAddress::RF24_IRQS;

        if (auto status = spiRead8(irqsReg); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            return status.value();
        }
    }

    etl::expected<void, Error> AT86RF215Chip::setBbcFskc0Config(
        Transceiver transceiver,
        BandwidthTimeProduct bt,
        ModIndexScale midxs,
        ModIndex midx,
        FskModOrder mord) {
        // Define the appropriate register for BBCn_FSKC0 based on the transceiver
        RegisterAddress regAddress = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKC0 : RegisterAddress::BBC1_FSKC0;

        uint8_t regValue = 0;
        regValue |= ((static_cast<uint8_t>(bt) & 0x03) << 6);    // BT: Bits [7:6]
        regValue |= ((static_cast<uint8_t>(midxs) & 0x03) << 4); // MIDXS: Bits [5:4]
        regValue |= ((static_cast<uint8_t>(midx) & 0x07) << 1);  // MIDX: Bits [3:1]
        regValue |= (static_cast<uint8_t>(mord) & 0x01);         // MORD: Bit [0]

        // Write the updated value back to the register
        return spiWrite8(regAddress, regValue);
    }

    etl::expected<void, Error> AT86RF215Chip::setBbcFskc1Config(
        Transceiver transceiver,
        FreqInversion freqInv,
        MrFskSymbolRate sr) {
        RegisterAddress fskc1 = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKC1 : RegisterAddress::BBC1_FSKC1;

        uint8_t regValue = 0;
        regValue |= static_cast<uint8_t>(freqInv) << 5;
        regValue |= static_cast<uint8_t>(sr);
        return spiWrite8(fskc1, regValue);
    }
    etl::expected<void, Error> AT86RF215Chip::setBbcFskc2Config(
        Transceiver transceiver,
        PreambleDetection preambleDet,
        ReceiverOverride recOverride,
        ReceiverPreambleTimeout recPreambleTimeout,
        ModeSwitchEnable modeSwitchEn,
        PreambleInversion preambleInversion,
        FecScheme fecScheme,
        InterleavingEnable interleavingEnable) {
        RegisterAddress fskc2 = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKC2 : RegisterAddress::BBC1_FSKC2;

        uint8_t regValue = 0x00;
        // Bit 7: PDTM - Preamble Detection Mode
        regValue |= (static_cast<uint8_t>(preambleDet) & 0x1) << 7;
        // Bits 6-5: RXO - Receiver Override
        regValue |= (static_cast<uint8_t>(recOverride) & 0x3) << 5;
        // Bit 4: RXPTO - Receiver Preamble Time Out
        regValue |= (static_cast<uint8_t>(recPreambleTimeout) & 0x1) << 4;
        // Bit 3: MSE - Mode Switch Enable
        regValue |= (static_cast<uint8_t>(modeSwitchEn) & 0x1) << 3;
        // Bit 2: PRI - Preamble Inversion
        regValue |= (static_cast<uint8_t>(preambleInversion) & 0x1) << 2;
        // Bit 1: FECS - FEC Scheme
        regValue |= (static_cast<uint8_t>(fecScheme) & 0x1) << 1;
        // Bit 0: FECIE - Interleaving Enable
        regValue |= (static_cast<uint8_t>(interleavingEnable) & 0x1) << 0;
        return spiWrite8(fskc2, regValue);
    }

    etl::expected<void, Error> AT86RF215Chip::setBbcFskc3Config(
        Transceiver transceiver,
        SfdDetectionThreshold sfdDetectionThreshold,
        PreambleDetectionThreshold preambleDetectionThreshold) {
        RegisterAddress fskc3 = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKC3 : RegisterAddress::BBC1_FSKC3;

        uint8_t regValue = 0x00;
        regValue |= (static_cast<uint8_t>(sfdDetectionThreshold) & 0xF) << 4;
        regValue |= (static_cast<uint8_t>(preambleDetectionThreshold) & 0xF) << 0;
        return spiWrite8(fskc3, regValue);
    }

    etl::expected<void, Error> AT86RF215Chip::setBbcFskc4Config(
        Transceiver transceiver,
        SfdQuantization sfdQuantization,
        Sfd32 sfd32,
        RawModeReversalBit rawModeReversal,
        CSFD1 csfd1,
        CSFD0 csfd0) {
        RegisterAddress regAddress = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKC4 : RegisterAddress::BBC1_FSKC4;

        uint8_t regValue = 0;
        regValue |= (static_cast<uint8_t>(sfdQuantization) & 0x1) << 6;
        regValue |= (static_cast<uint8_t>(sfd32) & 0x1) << 5;
        regValue |= (static_cast<uint8_t>(rawModeReversal) & 0x1) << 4;
        regValue |= (static_cast<uint8_t>(csfd1) & 0x3) << 2;
        regValue |= (static_cast<uint8_t>(csfd0) & 0x3) << 0;
        return spiWrite8(regAddress, regValue);
    }

    etl::expected<void, Error> AT86RF215Chip::setBbcFskphrtx(
        Transceiver transceiver,
        SfdUsed sfdUsed,
        DataWhitening dataWhitening) {
        RegisterAddress regAddress = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKPHRTX : RegisterAddress::BBC1_FSKPHRTX;

        uint8_t mask = (0x1 << 3) | (0x1 << 2);
        uint8_t val = ((static_cast<uint8_t>(sfdUsed) & 0x1) << 3) | ((static_cast<uint8_t>(dataWhitening) & 0x1) << 2);
        if (auto status = spiOverwriteBits(regAddress, mask, val); !status.has_value()) {
            return etl::unexpected(status.error());
        }

        return {};
    }

    etl::expected<void, Error> AT86RF215Chip::setBbcFskdm(
        Transceiver transceiver,
        FskPreamphasisEnable fskPreamphasisEnable,
        DirectModEnableFSKDM directModEnableFskdm) {
        RegisterAddress fskdm = transceiver == Transceiver::RF09 ? RegisterAddress::BBC0_FSKDM : RegisterAddress::BBC1_FSKDM;

        uint8_t regVal = (static_cast<uint8_t>(fskPreamphasisEnable) << 1) | static_cast<uint8_t>(directModEnableFskdm);
        return spiWrite8(fskdm, regVal);
    }

    etl::expected<void, Error> AT86RF215Chip::setTxDaci(Transceiver transceiver, bool inputEnable, uint8_t input) {
        RegisterAddress txDaci = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_TXDACI : RegisterAddress::RF24_TXDACI;

        uint8_t regVal = (static_cast<uint8_t>(inputEnable) << 7) | (input & 0x7F);
        return spiWrite8(txDaci, regVal);
    }

    etl::expected<void, Error> AT86RF215Chip::setTxDacq(Transceiver transceiver, bool inputEnable, uint8_t input) {
        RegisterAddress txDacq = transceiver == Transceiver::RF09 ? RegisterAddress::RF09_TXDACQ : RegisterAddress::RF24_TXDACQ;

        uint8_t regVal = (static_cast<uint8_t>(inputEnable) << 7) | (input & 0x7F);
        return spiWrite8(txDacq, regVal);
    }

    etl::expected<uint16_t, Error> AT86RF215Chip::getReceivedLength(Transceiver transceiver) {
        RegisterAddress regAddressLow;
        RegisterAddress regAddressHigh;

        // Determine the appropriate register addresses based on the transceiver
        if (transceiver == Transceiver::RF09) {
            regAddressLow = RegisterAddress::BBC0_RXFLL; // Replace with actual RF09 register address
            regAddressHigh = RegisterAddress::BBC0_RXFLH;
        } else {
            regAddressLow = RegisterAddress::BBC1_RXFLL; // Replace with actual RF24 register address
            regAddressHigh = RegisterAddress::BBC1_RXFLH;
        }
        uint8_t lowLengthByte;
        if (auto status = spiRead8(regAddressLow); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            lowLengthByte = status.value();
        }

        // Read the high-length byte
        uint8_t highLengthByte;
        if (auto status = spiRead8(regAddressHigh); !status.has_value()) {
            return etl::unexpected(status.error());
        } else {
            highLengthByte = status.value();
        }

        // Combine the bytes to form the received length
        return (static_cast<uint16_t>(highLengthByte & 0x07) << 8) | lowLengthByte;
    }
} // namespace AT86RF215
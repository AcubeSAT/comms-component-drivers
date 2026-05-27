#include "at86rf215GuardUtilities.hpp"
#include "at86rf215Definitions.hpp"

namespace AT86RF215 {
    bool AT86RF215Chip::MutexGuard::lockSpi() {
        if (ownsSpi) return false;

        if (xSemaphoreTake(chip.spiAccessMutexHandle, pdMS_TO_TICKS(SpiAccessMutexTimeoutMs)) == pdTRUE) {
            ownsSpi = true;
            return true;
        }
        return false;
    }

    void AT86RF215Chip::MutexGuard::unlockSpi() {
        if (ownsSpi) {
            xSemaphoreGive(chip.spiAccessMutexHandle);
            ownsSpi = false;
        }
    }

    bool AT86RF215Chip::MutexGuard::lockIqTx() {
        // Locking hierarchy: Cannot lock iqTx if we ALREADY own SPI
        if (ownsIqTx || ownsSpi) return false;

        if (xSemaphoreTake(chip.iqTxMutexHandle, pdMS_TO_TICKS(IqTxInterfaceAccessMutexDelayMs)) == pdTRUE) {
            ownsIqTx = true;
            return true;
        }
        return false;
    }

    void AT86RF215Chip::MutexGuard::unlockIqTx() {
        if (ownsIqTx) {
            xSemaphoreGive(chip.iqTxMutexHandle);
            ownsIqTx = false;
        }
    }

    bool AT86RF215Chip::MutexGuard::lockTransceiver(Transceiver transceiver) {
        if (transceiver == Transceiver::RF09) {
            // Locking hierarchy: Cannot lock 09 if we ALREADY own 24, iqTx, or SPI
            if (ownsRf09 || ownsRf24 || ownsIqTx || ownsSpi) return false;

            // locking must be prevented if the transceiver baseband core is currently receiving a packet
            bool transceiverBusy = false;
            taskENTER_CRITICAL();
            if (chip.basebandCoreIsReceiving09) {
                if ((xTaskGetTickCount() - chip.basebandCoreReceptionStartTime09) > pdMS_TO_TICKS(BasebandCorePacketReception09DelayMs)) {
                    chip.basebandCoreIsReceiving09 = false; // Delay expired, do not block the transceiver anymore
                } else {
                    transceiverBusy = true;
                }
            }
            taskEXIT_CRITICAL();

            if (transceiverBusy) {
                return false;
            }

            if (xSemaphoreTake(chip.transceiver09MutexHandle, pdMS_TO_TICKS(Radio09AccessMutexDelayMs)) == pdTRUE) {
                ownsRf09 = true;
                return true;
            }
        } else if (transceiver == Transceiver::RF24) {
            // Locking hierarchy: Cannot lock 24 if we ALREADY own iqTx or SPI
            if (ownsRf24 || ownsIqTx || ownsSpi) return false;

            // locking must be prevented if the transceiver baseband core is currently receiving a packet
            bool transceiverBusy = false;
            taskENTER_CRITICAL();
            if (chip.basebandCoreIsReceiving24) {
                if ((xTaskGetTickCount() - chip.basebandCoreReceptionStartTime24) > pdMS_TO_TICKS(BasebandCorePacketReception24DelayMs)) {
                    chip.basebandCoreIsReceiving24 = false; // Delay expired, do not block the transceiver anymore
                } else {
                    transceiverBusy = true;
                }
            }
            taskEXIT_CRITICAL();

            if (transceiverBusy) {
                return false;
            }

            if (xSemaphoreTake(chip.transceiver24MutexHandle, pdMS_TO_TICKS(Radio24AccessMutexDelayMs)) == pdTRUE) {
                ownsRf24 = true;
                return true;
            }
        }
        return false;
    }

    bool AT86RF215Chip::MutexGuard::lockAll() {
        // Prevent calling lockAll if we already hold anything
        if (ownsSpi || ownsRf09 || ownsRf24 || ownsIqTx) {
            return false;
        }

        // Lock in strict top-down order
        if (!lockTransceiver(Transceiver::RF09)) {
            return false;
        }

        if (!lockTransceiver(Transceiver::RF24)) {
            return false;
        }

        if (!lockIqTx()) {
            return false;
        }

        if (!lockSpi()) {
            return false;
        }

        return true;
    }

    AT86RF215Chip::MutexGuard::~MutexGuard() {
        // unlock in reverse order
        if (ownsSpi) {
            xSemaphoreGive(chip.spiAccessMutexHandle);
            ownsSpi = false;
        }

        if (ownsIqTx) {
            xSemaphoreGive(chip.iqTxMutexHandle);
            ownsIqTx = false;
        }

        if (ownsRf24) {
            xSemaphoreGive(chip.transceiver24MutexHandle);
            ownsRf24 = false;
        }

        if (ownsRf09) {
            xSemaphoreGive(chip.transceiver09MutexHandle);
            ownsRf09 = false;
        }
    }

    etl::expected<void, Error> AT86RF215Chip::DacOverrideSetup::setup() {
        if (auto status = chip.setStatePrivate(transceiver, State::RF_TRXOFF); !status.has_value()) {
            xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
        }

        // setup transceiver as shown in table 13-2
        if (chip.iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF ||
            (transceiver == Transceiver::RF09 && chip.iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ||
            (transceiver == Transceiver::RF24 && chip.iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09)) {
            // The respective baseband core is active. Transmit using CTX (continuous transmit)

            if ((transceiver == Transceiver::RF09 && !chip.basebandCoreConfig.continuousTransmit09) ||
                (transceiver == Transceiver::RF24 && !chip.basebandCoreConfig.continuousTransmit24)) {
                if (auto status = chip.spiOverwriteBits(pcReg, 0x80, 0x80); !status.has_value()) {
                    return etl::unexpected(status.error());
                } else {
                    pcValInitial = status.value();
                }
            }

            // Any frame length will do. It's value does not need to be restored in the destructor, as
            // the length is supposed to be re-written every time
            if (auto status = chip.spiWrite8(txfhlReg, 0x00); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            if (auto status = chip.spiWrite8(txfllReg, 0x10); !status.has_value()) {
                return etl::unexpected(status.error());
            }

            // Direct modulation must be disabled in order for the baseband core to have access to the I/Q DACs
            if ((transceiver == Transceiver::RF09 && static_cast<bool>(chip.txConfig.directModulation09)) ||
                (transceiver == Transceiver::RF24 && static_cast<bool>(chip.txConfig.directModulation24))) {
                if (auto status = chip.spiOverwriteBits(txdfeReg, 0x10, 0x00); !status.has_value()) {
                    return etl::unexpected(status.error());
                } else {
                    txdfeInitial = status.value();
                }
            }
        } else {
            // Transmit using only the radio. EEC needs to be temporarily turned off, in order to
            // be able to control TXPREP-TX transitions manually
            if (chip.iqInterfaceConfig.embeddedControlTX == EmbeddedControlTX::ENABLED) {
                if (auto status = chip.spiOverwriteBits(iqfc0Reg,  0x01, 0x00); !status.has_value()) {
                    return etl::unexpected(status.error());
                } else {
                    iqfc0ValInitial = status.value();
                }
            }
        }

        // override I DAC with maximum amplitude
        if ((transceiver == Transceiver::RF09 ? chip.txConfig.enableInputI09 : chip.txConfig.enableInputI24) == false ||
            (transceiver == Transceiver::RF09 ? chip.txConfig.dataInputI09 : chip.txConfig.dataInputI24) != 0x7E) {
            if (auto status =
                chip.spiOverwriteBits(txdaciReg,  0xFF, 0x80 | 0x7E); !status.has_value()) {
                return etl::unexpected(status.error());
            } else {
                txdaciInitial = status.value();
            }
        }

        // override Q DAC with zero amplitude
        if ((transceiver == Transceiver::RF09 ? chip.txConfig.enableInputQ09 : chip.txConfig.enableInputQ24) == false ||
            (transceiver == Transceiver::RF09 ? chip.txConfig.dataInputQ09 : chip.txConfig.dataInputQ24) != 0x3F) {
            if (auto status =
                chip.spiOverwriteBits(txdacqReg,  0xFF, 0x80 | 0x3F); !status.has_value()) {
                return etl::unexpected(status.error());
                } else {
                    txdacqInitial = status.value();
                }
        }

        return {};
    }

    AT86RF215Chip::DacOverrideSetup::~DacOverrideSetup() {
        // There are occasions where the api functions unlock the SPI mutex during lengthy processes, so
        // that the transceiver is not locked down. If said function experiences an error and needs to
        // return before relocking the SPI mutex, it is not safe for this destructor to use SPI.
        if (xSemaphoreGetMutexHolder(chip.spiAccessMutexHandle) != xTaskGetCurrentTaskHandle()) {
            xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
            return;
        }

        if (auto status = chip.setStatePrivate(transceiver, State::RF_TRXOFF); !status.has_value()) {
            xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
        }

        if (iqfc0ValInitial.has_value()) {
            if (auto status = chip.spiWrite8(iqfc0Reg, iqfc0ValInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
                return;
            }
        }

        if (pcValInitial.has_value()) {
            if (auto status = chip.spiWrite8(pcReg, pcValInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
                return;
            }
        }

        if (txdaciInitial.has_value()) {
            if (auto status = chip.spiWrite8(txdaciReg, txdaciInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
                return;
            }
        }

        if (txdacqInitial.has_value()) {
            if (auto status = chip.spiWrite8(txdacqReg, txdacqInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
            }
        }

        if (txdfeInitial.has_value()) {
            if (auto status = chip.spiWrite8(txdfeReg, txdfeInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
            }
        }
    }

    etl::expected<void, Error> AT86RF215Chip::SingleShotMeasurementSetup::setup() {
        if (auto status = chip.setStatePrivate(transceiver, State::RF_TRXOFF); !status.has_value()) {
            xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
        }

        if ((transceiver == Transceiver::RF09 && chip.basebandCoreConfig.baseBandEnable09) ||
            (transceiver == Transceiver::RF24 && chip.basebandCoreConfig.baseBandEnable24)) {
            if (auto status =
                chip.spiOverwriteBits(bbcPcReg, 0x04, 0x00); !status.has_value()) {
                return etl::unexpected(status.error());
            } else {
                bbcPcInitial = status.value();
            }
        }

        if (bw.has_value()) {
            if ((transceiver == Transceiver::RF09 && chip.rxConfig.receiverBandwidth09 != bw.value()) ||
                (transceiver == Transceiver::RF24 && chip.rxConfig.receiverBandwidth24 != bw.value())) {
                if (auto status =
                    chip.spiOverwriteBits(rxbwcReg, 0x0F, static_cast<uint8_t>(bw.value())); !status.has_value()) {
                    return etl::unexpected(status.error());
                } else {
                    rxbwcInitial = status.value();
                }
            }
        }

        return {};
    }

    AT86RF215Chip::SingleShotMeasurementSetup::~SingleShotMeasurementSetup() {
        // There are occasions where the api functions unlock the SPI mutex during lengthy processes, so
        // that the transceiver is not locked down. If said function experiences an error and needs to
        // return before relocking the SPI mutex, it is not safe for this destructor to use SPI.
        if (xSemaphoreGetMutexHolder(chip.spiAccessMutexHandle) != xTaskGetCurrentTaskHandle()) {
            xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
            return;
        }

        if (auto status = chip.setStatePrivate(transceiver, State::RF_TRXOFF); !status.has_value()) {
            xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
        }

        if (bbcPcInitial.has_value()) {
            if (auto status = chip.spiWrite8(bbcPcReg, bbcPcInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
                return;
            }
        }

        if (rxbwcInitial.has_value()) {
            if (auto status = chip.spiWrite8(rxbwcReg, rxbwcInitial.value()); !status.has_value()) {
                xEventGroupSetBits(chip.eventGroupHandle, ConfigDesynchronizationGroupBit);
            }
        }
    }
} // namespace AT86RF215

#pragma once

#include <utility>
#include <cstdint>
#include "stm32h7xx_hal_spi.h"
#include "etl/optional.h"
#include "etl/span.h"
#include "etl/expected.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "Logger.hpp"
#include "at86rf215definitions.hpp"
#include "at86rf215config.hpp"

namespace AT86RF215 {
    struct IrqStatus {
        etl::optional<uint8_t> rf09IrqsStatus;
        etl::optional<uint8_t> rf24IrqsStatus;
        etl::optional<uint8_t> bbc0IrqsStatus;
        etl::optional<uint8_t> bbc1IrqsStatus;
    };

    typedef struct {
        uint8_t dotDashMapping;  // 0bXX represents the dot-dash mapping (e.g., 0b01 for dot-dash)
        uint8_t dotDashNum;      // The number of symbols in the Morse code
    } MorseCodeMapping;

    static constexpr MorseCodeMapping getMorse(char c);

    enum class Error : uint8_t {
        FAILED_WRITING_TO_REGISTER,
        FAILED_READING_FROM_REGISTER,
        FAILED_CHANGING_STATE,
        UKNOWN_REQUESTED_STATE,
        INVALID_TRANSCEIVER_FREQ,
        INVALID_STATE_FOR_OPERATION,
        INVALID_PLL_CENTER_FREQ,
        INVALID_RSSI_MEASUREMENT,
        INVALID_AGC_CONTROl_WORD,
        ONGOING_TRANSMISSION_RECEPTION,
        MUTEX_LOCK_ERROR,
        TRANSMISSION_FAILED,
        RECEPTION_FAILED,
        SINGLE_SHOT_ENERGY_MEASUREMENT_FAILED,
        NULL_HANDLE,
        INVALID_CHIP_MODE,
        RX_WAIT_TIMEOUT,
        EMBEDDED_CONTROL_DISABLED,
        DESTINATION_BUFFER_TOO_SMALL,
        TX_BUFFER_TOO_LARGE,
        INVALID_REGISTER_VALUE,
        BASEBAND_OPERATION_FUNCTION_FAILED
    };

    inline uint8_t operator&(const uint8_t a, InterruptMask b) {
        return a & static_cast<uint8_t>(b);
    }

    class At86rf215_Utilities {
    public:
        /// Event group for signaling various events. Look in "at86rf215definitions" for interpretation
        /// of each group bit
        EventGroupHandle_t eventGroupHandle;

        /// Define here how long the transceiver should wait for certain events, before throwing an error (milliseconds).
        static constexpr uint16_t SpiAccessMutexTimeoutMs            = 100;
        static constexpr uint16_t SpiByteWriteCompleteDelayMs        = 100;
        static constexpr uint16_t SpiByteReadCompleteDelayMs         = 100;
        static constexpr uint16_t Radio09AccessMutexDelayMs          = 100;
        static constexpr uint16_t Radio24AccessMutexDelayMs          = 100;
        static constexpr uint16_t IqTxInterfaceAccessMutexDelayMs    = 100;
        static constexpr uint16_t IqPacketReception09DelayMs         = 100;
        static constexpr uint16_t IqPacketReception24DelayMs         = 100;
        static constexpr uint16_t TransceiverReadyDelayMs            = 100;
        static constexpr uint16_t BasebandTx09DelayMs                = 100;
        static constexpr uint16_t BasebandTx24DelayMs                = 100;
        static constexpr uint16_t EnergyDetCompletion09DelayMs       = 100;
        static constexpr uint16_t EnergyDetCompletion24DelayMs       = 100;

        At86rf215_Utilities() = default;

        /**
         * Initializer for AT86RF215 driver. This function must be called prior to performing
         * any operation with the transceiver.
         */
        etl::expected<void, Error> initializeResources(SPI_HandleTypeDef* spiHandle);

        /**
         * This method reads the transceiver interrupt code and takes any necessary actions.
         * It should be used inside a high priority freertos task, dedicated solely to transceiver irq handling.
         *
         * @returns If successful, a struct with the status of the interrupt registers
         *          (RF09_IRQS, RF24_IRQS, BBC0_IRQS, BBC1_IRQS)
         *
         */
        etl::expected<IrqStatus, Error> handleIrq();

        /**
         * Update the configuration structures.
         * @warning For the changes to apply, a subsequent call to chipReset() is required.
         */
        void setGeneralConfig(GeneralConfiguration&& GeneralConfig = GeneralConfiguration::defaultGeneralConfig()) {
            generalConfig = std::move(GeneralConfig);
        }
        void setRXConfig(RXConfig&& RXConfig = RXConfig::DefaultRXConfig()) {
            rxConfig = std::move(RXConfig); // Move the new config into rxConfig
        }
        void setTXConfig(TXConfig&& TXConfig = TXConfig::defaultTXConfig()) {
            txConfig = std::move(TXConfig); // Move the new config into rxConfig
        }
        void setBaseBandCoreConfig(BasebandCoreConfig&& BasebandCoreConfig = BasebandCoreConfig::defaultBasebandCoreConfig()) {
            basebandCoreConfig = std::move(BasebandCoreConfig); // Move the new config into rxConfig
        }
        void setFrequencySynthesizerConfig(FrequencySynthesizerConfig&& FrequencySynthesizer = FrequencySynthesizerConfig::defaultFrequencySynthesizerConfig()) {
            freqSynthesizerConfig = std::move(FrequencySynthesizer); // Move the new config into rxConfig
        }
        void setExternalFrontEndControlConfig(ExternalFrontEndConfig&& ExternalFrontEndConfig = ExternalFrontEndConfig::defaultExternalFrontEndConfig()) {
            externalFrontEndConfig = std::move(ExternalFrontEndConfig);
        }
        void setInterruptConfig(BasebandCoreInterruptsConfig&& InterruptsConfig = BasebandCoreInterruptsConfig::defaultBasebandCoreInterruptsConfig()) {
            basebandCoreInterruptsConfig = std::move(InterruptsConfig);
        }
        void setRadioInterruptConfig(RadioInterruptsConfig&& RadioInterruptsConfig = RadioInterruptsConfig::defaultRadioInterruptsConfig()) {
            radioInterruptsConfig = std::move(RadioInterruptsConfig);
        }
        void setIQInterfaceConfig(IQInterfaceConfig&& IQInterfaceConfig = IQInterfaceConfig::defaultIQInterfaceConfig()) {
            iqInterfaceConfig = std::move(IQInterfaceConfig);
        }

        /**
         * Fetches the current state of the transceiver
         * @note Mutex protected wrapper for get_state_private()
         *
         * @param transceiver	Specifies the transceiver used
         */
        etl::expected<State, Error> getState(Transceiver transceiver);

        /**
         * Sets the state of the transceiver
         * @note Mutex protected wrapper for set_state_private()
         *
         * @param transceiver	Specifies the transceiver used
         * @param state_cmd		Command responsible for changing the state
         */
        etl::expected<void, Error> setState(Transceiver transceiver, State state_cmd);

        /**
         * Does chip reset and reads from the interrupt status registers via SPI, resetting them.
         * It also restores the config settings
         */
        etl::expected<void, Error> chipReset();

        /**
         * Try to read something from the transceiver to ensure the spi connection works
         */
        etl::expected<void, Error> checkTransceiverConnection();

        /**
         * Use the logger to print the current state of the transceiver
         */
        etl::expected<void, Error> printState(Transceiver transceiver);

        /**
         * Print an error using the logger
         */
        static void printError(Error& err);

        /**
         * Single shot measurement of power in the specified bandwidth, around the
         * set central frequency.
         * @param transceiver       Selected transceiver
         * @param bw                Power will be measured in this bandwidth. If no value is given, the
         *                          measurement will take place in the already set bandwidth.
         * @returns                 The average power in dBm. The range of possible values is -127..4 dBm
         */
        etl::expected<int8_t, Error> singleShotEnergyMeasurement(Transceiver transceiver, etl::optional<ReceiverBandwidth> bw);

        /**
         *  Start transmitting a carrier wave at the config frequency
         *
         *  @param transmissionTimeMs: How long to transmit the carrier, in ms. When the time elapses, the original
         *                             configuration is restored.
         *  @warning Use this function for debugging only
         */
        etl::expected<void, Error> transmitCarrier(Transceiver transceiver, uint32_t transmissionTimeMs);

        /**
         * Transmit a packet using the baseband core.
         *
         * @param transceiver		Specifies the transceiver used
         * @param packet			The packet data. The size must be smaller than the maximum packet length, which is
         *                          2047
         */
        etl::expected<void, Error> packetTransmissionBaseband(Transceiver transceiver, etl::span<uint8_t> packet);

        /**
         * Set the receiver to a "listening" state, so that packet reception through the
         * baseband core may be performed.
         *
         * @note This function essentially sets the transceiver to state RX, but the user is
         *       not stopped from performing an energy measurement, or a tx operation (either with
         *       the baseband core or through the I/Q interface), after this function returns.
         *       The function has to be called again to re-enter the "listening" state.
         *
         *
         * @param transceiver		Specifies the transceiver used
         * @param destBuff          A user provided buffer to write the packet. In order to guarantee
         *                          that there will be no buffer overflow, it's capacity should be
         *                          at least 2047 (the maximum supported packet length)
         */
        etl::expected<void, Error> preparePacketReceptionBaseband(Transceiver transceiver, etl::span<uint8_t> destBuff);

        /**
         * Waits for packet reception. The packet is written to the registered buffer from
         * the preparePacketReceptionBaseband() call.
         *
         * @param timeoutDelayMs Defines how long the function should wait for a packet before it returns.
         *
         * @note A 'RX_WAIT_TIMEOUT' error will be returned if the functions returns because of timeout
         *
         * @note The actual copying of the reception packet happens in handleIrq(), when a "receiver frame
         *       end interrupt" arrives. All this function does is return the packet length, once the reception is
         *       complete.
         * @returns The received packet length. In case of an error, 0 is returned.
         */
        etl::expected<uint16_t, Error> waitForPacketReceptionBaseband(Transceiver transceiver, uint32_t timeoutDelayMs);

        /**
         * Transmit a packet through the Tx I/Q interface, when embedded control is active
         *
         * @param basebandOp: This function is passed by the user and is responsible for for baseband processing and
         *                    sending packets through the I/Q interface. Two conditions must be met:
         *                    - The function returns a boolean (true on success, false on failure)
         *                    - The function returns only when it is confirmed that packet transmission is complete
         *
         * @note The design choice of passing a processing function inside the driver has to do with the fact that
         *       the respective radio and the singular Tx I/Q interface have to be locked, in order to avoid concurrent
         *       resource usage. Therefore, the driver has to know when the baseband processing is finished, so said
         *       resources are unlocked.
         */
        template <typename BasebandOp>
        etl::expected<void, Error> packetTransmissionIQEmbeddedControl(Transceiver transceiver, BasebandOp basebandOp);

        /**
         * Set the transceiver to a "listening" state , so that packet reception through the
         * I/Q interface may be performed.
         *
         * @note This function essentially sets the transceiver to state RX, but the user is
         *       not stopped from performing a Tx operation (either with
         *       the baseband core or through the I/Q interface). This function
         *       has to be called again to re-enter the "listening" state.
         */
        etl::expected<void, Error> preparePacketReceptionIQ(Transceiver transceiver);

        /**
         * Wait for packet reception through the I/Q interface.
         *
         * @param timeoutDelayMs Defines how long the function should wait for a packet before it returns.
         *
         * @note An 'RX_WAIT_TIMEOUT' error will be returned if the functions returns because of timeout
         *
         * @note The user needs to take the following actions externally:
         *    - set the iqPreambleReception event bit immediately after the external baseband processor
         *      detects a preamble, so that the transceiver is locked (transceiverOccupied event bit set) and the
         *      transceiver's internal AGC frozen.
         *
         *    - set the iqPacketReception event bit once the external baseband processor fully received the
         *      packet, so that the AGC is released and the transceiverOccupied event bit is reset
         */
        etl::expected<void, Error> waitForPacketReceptionIQ(Transceiver transceiver, uint32_t timeoutDelayMs);

        /**
         * DEBUG FUNCTION: Set the transceiver to a state where incoming data in the LDVS interface is looped back
         *                 (directly from the receiver, TXD, to the RX09 and RX24 drivers). There is a delay of
         *                 up to 18 "bit periods" between the incoming and the looped back data streams.
         *
         * @note For the IQ interface to work, any chip mode that is not RF_MODE_BBRF should be selected:
         *               RF_MODE_RF:  I/Q IF enabled
         *               RF_MODE_BBRF09: I/Q IF enabled (sub 1GHz)
         *               RF_MODE_BBRF24: I/Q IF enabled (2.4GHz)
         */
        etl::expected<void, Error> enableIQLoopbackMode();

        /**
         * DEBUG FUNCTION: Disable LVDS interface loopback.
         */
        etl::expected<void, Error> disableIQLoopbackMode();

        /**
         * Transmit a sequence of characters encoded as morse code, with on-off keying modulation (OOK).
         * This is achieved using the "DAC overwrite"  features (section 13.1.2), which allows transmission
         * of a pure LO carrier.
         *
         * @param wpm Words per minute. This function cannot handle sub millisecond (or close to millisecond)
         *            symbol durations. Enter a reasonable value, that is well below 1200 words per minute.
         *
         * @details
         * 1 time unit : 1200/wpm milliseconds
         * dot duration: 1 time unit
         * dash duration: 3 time units
         * duration between elements of the same character: 1 time unit
         * duration between characters: 3 time units
         * duration between words: 7 time units
         */
        etl::expected<void, Error> transmitMorseCode(
            Transceiver transceiver,
            float wpm,
            etl::string_view sequence);

    private:
        /// Mutex for protecting against concurrent access to spi and radio resources
        StaticSemaphore_t spiAccessMutexBuffer = {};
        SemaphoreHandle_t spiAccessMutexHandle;

        StaticSemaphore_t transceiver09MutexBuffer = {};
        SemaphoreHandle_t transceiver09MutexHandle;

        StaticSemaphore_t transceiver24MutexBuffer = {};
        SemaphoreHandle_t transceiver24MutexHandle;

        StaticSemaphore_t iqTxMutexBuffer = {};
        SemaphoreHandle_t iqTxMutexHandle;

        /**
         * Mutex locking utility following a RAII like pattern. To avoid deadlocks, the locking order must strictly
         * be:
         *
         * - transceiver09MutexHandle
         * - transceiver24MutexHandle
         * - iqTxMutexHandle
         * - spiAccessMutexHandle
         *
         * with unlocking order being the opposite.
         *
         * @note All lock functions return true if the mutexes are locked
         * successfully. If false is returned, locking failed because of timeout or invalid lock order. The caller
         * should always return if a lock fails, so the MutexGuard destructor is called to unlock any remaining
         * mutexes.
         */
        class MutexGuard {
        public:
            MutexGuard(SemaphoreHandle_t spiAccessMutexHandle,
                       SemaphoreHandle_t transceiver09MutexHandle,
                       SemaphoreHandle_t transceiver24MutexHandle,
                       SemaphoreHandle_t iqTxMutexHandle) :
                spiAccessMutexHandle(spiAccessMutexHandle),
                transceiver09MutexHandle(transceiver09MutexHandle),
                transceiver24MutexHandle(transceiver24MutexHandle),
                iqTxMutexHandle(iqTxMutexHandle) {}

            MutexGuard(const MutexGuard&) = delete;
            MutexGuard& operator=(const MutexGuard&) = delete;

            bool lockSpi() {
                if (ownsSpi) return false;

                if (xSemaphoreTake(spiAccessMutexHandle, pdMS_TO_TICKS(SpiAccessMutexTimeoutMs)) == pdTRUE) {
                    ownsSpi = true;
                    return true;
                }
                return false;
            }

            void unlockSpi() {
                if (ownsSpi) {
                    xSemaphoreGive(spiAccessMutexHandle);
                    ownsSpi = false;
                }
            }

            bool lockIqTx() {
                // Locking hierarchy: Cannot lock iqTx if we ALREADY own SPI
                if (ownsIqTx || ownsSpi) return false;

                if (xSemaphoreTake(iqTxMutexHandle, pdMS_TO_TICKS(IqTxInterfaceAccessMutexDelayMs)) == pdTRUE) {
                    ownsIqTx = true;
                    return true;
                }
                return false;
            }

            void unlockIqTx() {
                if (ownsIqTx) {
                    xSemaphoreGive(iqTxMutexHandle);
                    ownsIqTx = false;
                }
            }

            bool lockTransceiver(Transceiver transceiver) {
                if (transceiver == Transceiver::RF09) {
                    // Locking hierarchy: Cannot lock 09 if we ALREADY own 24, iqTx, or SPI
                    if (ownsRf09 || ownsRf24 || ownsIqTx || ownsSpi) return false;

                    if (xSemaphoreTake(transceiver09MutexHandle, pdMS_TO_TICKS(Radio09AccessMutexDelayMs)) == pdTRUE) {
                        ownsRf09 = true;
                        return true;
                    }
                } else if (transceiver == Transceiver::RF24) {
                    // Locking hierarchy: Cannot lock 24 if we ALREADY own iqTx or SPI
                    if (ownsRf24 || ownsIqTx || ownsSpi) return false;

                    if (xSemaphoreTake(transceiver24MutexHandle, pdMS_TO_TICKS(Radio24AccessMutexDelayMs)) == pdTRUE) {
                        ownsRf24 = true;
                        return true;
                    }
                }
                return false;
            }

            bool lockAll() {
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

            ~MutexGuard() {
                // unlock in reverse order
                if (ownsSpi) {
                    xSemaphoreGive(spiAccessMutexHandle);
                    ownsSpi = false;
                }
                if (ownsIqTx) {
                    xSemaphoreGive(iqTxMutexHandle);
                    ownsIqTx = false;
                }
                if (ownsRf24) {
                    xSemaphoreGive(transceiver24MutexHandle);
                    ownsRf24 = false;
                }
                if (ownsRf09) {
                    xSemaphoreGive(transceiver09MutexHandle);
                    ownsRf09 = false;
                }
            }

        private:
            SemaphoreHandle_t spiAccessMutexHandle;
            SemaphoreHandle_t transceiver09MutexHandle;
            SemaphoreHandle_t transceiver24MutexHandle;
            SemaphoreHandle_t iqTxMutexHandle;

            bool ownsSpi = false;
            bool ownsRf09 = false;
            bool ownsRf24 = false;
            bool ownsIqTx = false;
        };

        /// Event group for signaling various events
        StaticEventGroup_t eventGroupBuffer;

        /// SPI handle
        SPI_HandleTypeDef* hspi;

        /// Structs with register configurations
        GeneralConfiguration generalConfig;
        RXConfig rxConfig;
        TXConfig txConfig;
        BasebandCoreConfig basebandCoreConfig;
        FrequencySynthesizerConfig freqSynthesizerConfig;
        ExternalFrontEndConfig externalFrontEndConfig;
        BasebandCoreInterruptsConfig basebandCoreInterruptsConfig;
        RadioInterruptsConfig radioInterruptsConfig;
        IQInterfaceConfig iqInterfaceConfig;

        /// User provided memory for storing a received packet in baseband core operation
        etl::span<uint8_t> destBuffer09;
        etl::span<uint8_t> destBuffer24;

        /// Received packet's length in baseband core operation
        uint16_t receivedPacketLength09;
        uint16_t receivedPacketLength24;

        /**
         * Writes a byte to a specified address
         *
         * @param address	Specifies the address to write to
         * @param value		The value to write to the specified address
         */
        etl::expected<void, Error> spiWrite8(RegisterAddress address, uint8_t value);

        /**
         * Reads a byte to a specified address
         *
         * @param address	Specifies the address to read from
         * @returns 		Returns the read byte
         */
        etl::expected<uint8_t, Error> spiRead8(RegisterAddress address);

        /**
         * Writes a byte to a specified address
         *
         * @param address	Specifies the address to start writing to
         * @param value		Values to write to address
         */
        etl::expected<void, Error> spiBlockWrite8(RegisterAddress address, etl::span<uint8_t> value);

        /**
         * Reads a byte to a specified address. Assumes that the caller has
         * allocated the expected memory.
         *
         * @param address	Specifies the address to start reading from
         * @param response	Buffer to place the read bytes
         */
        etl::expected<void, Error> spiBlockRead8(RegisterAddress address, etl::span<uint8_t> response);

        /**
         * Apply a mask to a specified address, using bitwise OR
         *
         * @param address	Specifies the address to write to
         * @param mask		The mask to apply to the specified address
         */
        etl::expected<void, Error> spiApplyBitwiseOr(RegisterAddress address, uint8_t mask);

        /**
         * Apply a mask to a specified address, using bitwise AND
         *
         * @param address	Specifies the address to write to
         * @param mask		The mask to apply to the specified address
         */
        etl::expected<void, Error> spiApplyBitwiseAnd(RegisterAddress address, uint8_t mask);

        /**
         * Fetches the current state of the transceiver
         *
         * @param transceiver	Specifies the transceiver used
         */
        etl::expected<State, Error> getStatePrivate(Transceiver transceiver);

        /**
         * Sets the state of the transceiver
         *
         * @param transceiver	Specifies the transceiver used
         * @param stateCmd		Command responsible for changing the state
         */
        etl::expected<void, Error> setStatePrivate(Transceiver transceiver, State stateCmd);

        /**
         * Sets PLL channel spacing (25kHz resolution)
         *
         * @param transceiver	Specifies the transceiver used
         * @param spacing	Configures the channel spacing with a resolution of 25kHz
         */
        etl::expected<void, Error> setPllChannelSpacing(Transceiver transceiver, uint8_t spacing);

        /**
         * Gets PLL channel spacing
         * @param transceiver	Specifies the transceiver used
         */
        etl::expected<uint8_t, Error> getPllChannelSpacing(Transceiver transceiver);

        /**
         * Sets the central channel frequency of the PLL
         *
         * @param transceiver	Specifier the transceiver used
         * @param freq 			Central frequency of the PLL
         */
        etl::expected<void, Error> setPllChannelFrequency(Transceiver transceiver, uint16_t freq);

        /**
         * Fetches the central channel frequency of the PLL
         *
         * @param transceiver	Specifier the transceiver used
         */
        etl::expected<uint16_t, Error> getPllChannelFrequency(Transceiver transceiver);

        /**
         * Gets the channel number of the PLL
         *
         * @param transceiver	Specifier the transceiver used
         */
        etl::expected<uint16_t, Error> getPllChannelNumber(Transceiver transceiver);

        /**
         * Sets the loop bandwitdh of the PLL. Options are:
         * 	- Default (0x0)
         * 	- 15% smaller than default (0x1)
         * 	- 15% larger than default (0x2)
         * 	This is only applicable to the RF09 transceiver
         *
         * @param bw	Loopbandwidth of PLL
         */
        etl::expected<void, Error> setPllBw(PLLBandwidth bw);

        /**
         * Gets the loop bandwitdh of the PLL. Options are:
         * 	- Default
         * 	- 15% smaller than default
         * 	- 15% larger than default
         * 	This is only applicable to the RF09 transceiver
         *
         * @returns 	PLL bandwidth
         */
        etl::expected<PLLBandwidth, Error> getPllBw();

        /**
         * Gets the state of the PLL (locked/not locked)
         *
         * @param transceiver		Specify the transceiver used
         */
        etl::expected<PLLState, Error> getPllState(Transceiver transceiver);

        /**
         * Configures the PLL
         *
         * @param transceiver		         Specify the transceiver used
         * @param frequencySynthesizerConfig Reference to configuration with frequency, channel mode and bandwidth
         */
        etl::expected<void, Error> configurePll(
            Transceiver transceiver,
            FrequencySynthesizerConfig& frequencySynthesizerConfig);

        /**
         * Gets the part number of the device
         *
         * @returns 	The part number that is one of the following:
         * 					- AT86RF215
         * 					- AT86RF215IQ
         * 					- AT86RF215M
         */
        etl::expected<DevicePartNumber, Error> getPartNumber();

        /**
         * Gets the version number of the device
         *
         */
        etl::expected<DeviceVersionNumber, Error> getVersionNumber();

        /**
         * Sets the PLL frequency
         *
         * @param transceiver	Specify the transceiver used
         * @param freq			PLL frequency
         */
        etl::expected<void, Error> setPllFrequency(Transceiver transceiver, uint8_t freq);

        /**
         * Gets the PLL frequency
         *
         * @param transceiver	Specify the transceiver used
         * @return 				PLL frequency
         */
        etl::expected<uint8_t, Error> getPllFrequency(Transceiver transceiver);

        /**
         * Sets trimming capacitor to match the load of external TCXO (if used), with
         * a precision of 0.3 pF.
         *
         * C_L = 0.5*(C_X + C_TRIM + C_PAR)
         *
         * Where:
         * 	- C_L:		Load of the crystal
         * 	- C_X: 		External capacitor
         * 	- C_TRIM:	Trimming capacitor
         *	- C_PAR:	Parasitic capacitor
         *
         * @param trim	Crystal trimming (0.3 pF precision)
         */
        etl::expected<void, Error> setTcxoTrimming(CrystalTrim trim);

        /**
         * Reads trimming capacitor to match the load of external TXCO (if used), with
         * a precision of 0.3 pF.
         *
         */
        etl::expected<CrystalTrim, Error> readTcxoTrimming();

        /**
         * Set fast start-up enable option for external crystal oscillator
         * If enabled, it will increase start-up time by 0.8mA while also increasing
         * the start-up time.
         *
         * @param fastStartUp		Fast start-up option for TCXO
         */
        etl::expected<void, Error> setTcxoFastStartUpEnable(bool fastStartUp);

        /**
         * Reads fast start-up enable option for external crystal oscillator
         * If enabled, it will increase start-up time by 0.8mA while also increasing
         * the start-up time.
         *
         */
        etl::expected<bool, Error> readTcxoFastStartUpEnable();

        /**
         * Set PA ramp-up time in TX chain.
         *
         * Longer ramp-up time requires more power but decreases possible spurious emissions
         *
         * @param transceiver		Specifies the transceiver used
         * @return 					PA ramp-up time
         */
        etl::expected<PowerAmplifierRampTime, Error> getPaRampUpTime(Transceiver transceiver);
        /**
         * Get the low pass cut-off frequency of the filter in the TX chain.
         * For the filter response refer to Figure 6-2, Atmel AT86RF215 datasheet
         *
         * @param transceiver		Specifies the transceiver used
         * @return 					Filter cutoff frequency
         */
        etl::expected<TransmitterCutOffFrequency, Error> getCutoffFreq(Transceiver transceiver);

        /**
         * Get the relative cut-off frequency of the filter in the TX chain.
         *
         * @param transceiver		Specifies the transceiver used
         * @return 					Filter cutoff frequency
         */
        etl::expected<TxRelativeCutoffFrequency, Error> getRelativeCutoffFreq(Transceiver transceiver);
        /**
         * Get whether direct modulation is used in the TX chain.
         * Only available for baseband FSK and OQPSK)
         *
         * @param transceiver		Specifies the transceiver used
         * @return 					Indicates whether direct modulation is used
         */
        etl::expected<bool, Error> getDirectModulation(Transceiver transceiver);

        /**
         * Set the sample rate of the receiver.
         * For exact configuration of the sample_rate refer to AT86RF215 datasheet, Table 6-6
         * or in registers.h*
         *
         * @param transceiver		Specifies the transceiver used
         * @return 					Sample rate of receiver
         */
        etl::expected<ReceiverSampleRate, Error> getSampleRate(Transceiver transceiver);

        /**
         * Read PA DC current
         *
         * @param transceiver		Specifies the transceiver used
         * @return 					PA DC current
         */
        etl::expected<PowerAmplifierCurrentControl, Error> getPaDcCurrent(Transceiver transceiver);

        /**
         * Get whether the external LNA is bypassed
         *
         * @param transceiver		Specifies the transceiver used
         * @return					Get whether external LNA is bypassed
         */
        etl::expected<bool, Error> getLnaBypassed(Transceiver transceiver);

        /**
         * Shows whether Automatic Gain Control is used for the external LNA.
         *
         * @param transceiver		Specifies the transceiver used
         * @return agcmap			AGC gain
         */
        etl::expected<AutomaticGainControlMAP, Error> getAgcmap(Transceiver transceiver);

        /**
         * Set whether an external analog voltage is supplied to AVDD0 or AVDD1 for the sub-1 GHz
         * and the 2.4 Ghz transceiver respectively
         *
         * @param transceiver		Specifies the transceiver used
         * @return					Specifies whether external voltage is supplied to AVDD
         */
        etl::expected<AutomaticVoltageExternal, Error> getExternalAnalogVoltage(Transceiver transceiver);

        /**
         * Shows whether analog voltage is settled
         *
         * @param transceiver		Specifies the transceiver used
         * @return					Specifies whether AV is settled
         */
        etl::expected<bool, Error> getAnalogVoltageSettledStatus(Transceiver transceiver);

        /**
         * Fetches supplied voltage of the internal PA
         *
         * @param transceiver		Specifies the transceiver used
         * @return					PA supplied voltage
         */
        etl::expected<PowerAmplifierVoltageControl, Error> getAnalogPowerAmplifierVoltage(Transceiver transceiver);

        /**
         * Set receiver energy detection average duration given by df*dtb
         *
         * @param transceiver		Specifies the transceiver used
         * @param df				Detection factor
         * @param dtb				Detection time scale
         */
        etl::expected<void, Error> setEdAverageDetection(
            Transceiver transceiver,
            uint8_t df,
            EnergyDetectionTimeBasis dtb);

        /**
         * Read receiver energy detection average duration given by df*dtb in μs
         *
         * @param transceiver		Specifies the transceiver used
         */
        etl::expected<uint16_t, Error> getEdAverageDetection(Transceiver transceiver);

        etl::expected<int8_t, Error> getReceiverEnergyDetection(Transceiver transceiver);


        /**
         * Set transceiver battery monitor status
         *
         * @param status			Battery monitor status
         */
        etl::expected<void, Error> setBatteryMonitorStatus(bool status);

        /**
         * Get transceiver battery monitor status
         *
         * @return status			Battery monitor status
         */
        etl::expected<BatteryMonitorStatus, Error> getBatteryMonitorStatus();

        /**
         * Set the threshold of the battery monitoring range (low/high)
         *
         * @param range				Transceiver battery range
         */
        etl::expected<void, Error> setBatteryMonitorHighRange(BatteryMonitorHighRange range);

        /**
         * Gets the threshold of the battery monitoring range (low/high)
         *
         * @return range			Transceiver battery monitoring range
         */
        etl::expected<uint8_t, Error> getBatteryMonitorHighRange();

        /**
         * Sets voltage threshold for battery monitoring
         *
         * @param threshold			Battery voltage threshold
         */
        etl::expected<void, Error> setBatteryMonitorVoltageThreshold(BatteryMonitorVoltageThreshold threshold);
        etl::expected<void, Error> setBatteryMonitorControl(
            BatteryMonitorHighRange range,
            BatteryMonitorVoltageThreshold threshold);

        /**
         * Get voltage threshold for battery monitoring
         *
         * @return threshold		Battery voltage threshold
         */
        etl::expected<uint8_t, Error> getBatteryMonitorVoltageThreshold();

        /**
         * Sets up the target registers for setting up the transceiver tx frontend
         *
         * @param transceiver		Specifies the transceiver used
         * @param paRampTime	    TX PA ramp time
         * @param cutoff 			TX filter cut-off frequency
         * @param txRelCutoff     TX relative cut-off frequency
         * @param directMod		Specifies whether direct modulation is supported (supported for FSK and OQPSK)
         * @param txSampleRate    TX sample rate
         * @param paCurrControl 	Controls power amplifier current reduction
         * @param transceiver		Specifies the transceiver used
         * @param txOutPower		Output power of the transmitter (0x00-0x1F in 1dB steps)
         * @param extLnaBypass 	Specifies whether external LNA will be bypassed
         * @param agcMap			Controls gain of the gain controler for the external LNA
         * @param avgExt			Disables internal supply voltage
         * @param avEnable			Defines whether voltage regulator is enabled during TRXOFF
         * @param paVcontrol		Controls supply voltage of internal PA
         */
        etl::expected<void, Error> setupTxFrontend(
            Transceiver transceiver,
            PowerAmplifierRampTime paRampTime,
            TransmitterCutOffFrequency cutoff,
            TxRelativeCutoffFrequency txRelCutoff,
            Direct_Mod_Enable_FSKDM directMod,
            TransmitterSampleRate txSampleRate,
            PowerAmplifierCurrentControl paCurrControl,
            uint8_t txOutPower,
            ExternalLNABypass extLnaBypass,
            AutomaticGainControlMAP agcMap,
            AutomaticVoltageExternal avgExt,
            AnalogVoltageEnable avEnable,
            PowerAmplifierVoltageControl paVcontrol,
            ExternalFrontEndControl externalFrontEndControl);

        /**
         * Sets up the target registers for setting up the transceiver rx frontend
         *
         * @param transceiver		Specifies the transceiver used
         * @param ifInversion		Defines whether IF inverted signal is used in the receive side
         * @param ifShift			If true, it shifts the IF frequency by a factor of 1.25
         * @param rxBw				Specifies the receiver bandwidth
         * @param rxRelCutoff		RX filter relative cut-off frequency
         * @param rxSampleRate	RX sample rate
         * @param agcInput			If true, the filtered front signal is used rather than the signal before the channel filter
         * @param agcAvgSample	AGC averaging
         * @param agcEnable 		If set to true AGC is enabled, otherwise, the gain is defined by the agc_gain parameter (AGCS.GCW register)
         * @param agcTarget		Sets the target output gain of the AGC
         * @param gainControlWord	If AGC is not enabled, then this register is used to define the maximum gain (valid values 0-23 with 3dB steps)
         */
        etl::expected<void, Error> setupRxFrontend(
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
            uint8_t gainControlWord);

        /**
         * Set up IQ interface
         *
         * @param externalLoop		Defines whether external loopback is enabled (for testing purposes only)
         * @param outCur			Defines output current
         * @param commonModeVol	Voltage of I/Q signals
         * @param commonModeIee	Whether voltage of I/Q signals is set to 1V2 (IEEE Std 1596-compliant)
         * @param embeddedTxStart	Specifies whether a control bit is automatically transmitted upon start and finish of IQ stream
         * @param chipMode			Defines what operates out of the baseband core and I/Q IF
         * @param skewAlignment	Specifies the alignment of I/Q data relative to the clock edges of RXCLK
         */
        etl::expected<void, Error> setupIq(
            ExternalLoopback externalLoop,
            IQOutputCurrent outCur,
            IQmodeVoltage commonModeVol,
            IQmodeVoltageIEE commonModeIee,
            EmbeddedControlTX embeddedTxStart,
            ChipMode chipMode,
            SkewAlignment skewAlignment);

        /**
         *  Identify whether the IQ interface deserializer is synchronized
         */
        etl::expected<bool, Error> getIqSyncStatus();

        /**
         * Sets up parameters for received energy tracking
         *
         * @param transceiver				Specifies the transceiver used
         * @param energyMode				Energy detection measurement mode (AUTO/Single/Continuous/Off)
         * @param energyDetectFactor		Duration factor over which the results will be averaged (mult by time base)
         * @param energyTimeBasis			Time basis multiplied by the detection factor to determine the averaging window
         */
        etl::expected<void, Error> setupRxEnergyDetection(
            Transceiver transceiver,
            EnergyDetectionMode energyMode,
            uint8_t energyDetectFactor,
            EnergyDetectionTimeBasis energyTimeBasis);

        /**
         * Sets up internal crystal oscillator
         *
         * @param fast_start_up				Fast start-up option for TCXO (quicker start-up at the expense of current consumption)
         * @param crystal_trim				Controls trim-capacitor to match load capacitance of external oscillator
         */
        etl::expected<void, Error> setupCrystal(bool fast_start_up, CrystalTrim crystal_trim);

        /**
         * Sets up IRQ behavior
         *
         * @param maskMode				Defines whether reasons for IRQ call appear in IRQS register
         * @param irqPolarity				Sets up the IRQ pin polarity (active high or low)
         * @param padDriverStrength		Driver strength (mA) of MISO, IRQ and FEA/FEB pins
         */
        etl::expected<void, Error> setupIrqCfg(
            bool maskMode,
            IRQPolarity irqPolarity,
            PadDriverStrength padDriverStrength);

        /**
         * Sets up physical baseband
         *
         * @param transceiver			Specifies the transceiver used
         * @param continuousTransmit 	Transmission continues for as long as PC.CTX is set
         * @param frameSeqFilter		Successful frame reception IRQ is only triggered if the frame's FCS is valid
         * @param transmitterAutoFCS	Define whether the FCS is inserted automatically to the PSU
         * @param fcsType				16- or 32-bit FCS
         * @param basebandEnable		Sets whether the baseband is enabled (as opposed to the radio mode)
         * @param phyType				Defines the physical layer type
         */
        etl::expected<void, Error> setupPhyBaseband(
            Transceiver transceiver,
            bool continuousTransmit,
            bool frameSeqFilter,
            bool transmitterAutoFCS,
            FrameCheckSequenceType fcsType,
            bool basebandEnable,
            PhysicalLayerType phyType);

        etl::expected<void, Error> setupIrqMask(
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
            bool receiverFrameStart);

        /**
         *
         * Returns the IRQ register from the corresponding transceiver
         *
         * @param transceiver		Target transceiver
         */
        etl::expected<uint8_t, Error> getIrq(Transceiver transceiver);

        etl::expected<void, Error> setBbcFskc0Config(
            Transceiver transceiver,
            Bandwidth_time_product bt,
            Mod_index_scale midxs,
            Mod_index midx,
            FSK_mod_order mord);

        etl::expected<void, Error> setBbcFskc1Config(
            Transceiver transceiver,
            Freq_Inversion freqInv,
            MR_FSK_symbol_rate sr);

        etl::expected<void, Error> setBbcFskc2Config(
            Transceiver transceiver,
            Preamble_Detection preambleDet,
            Receiver_Override recOverride,
            Receiver_Preamble_Timeout recPreambleTimeout,
            Mode_Switch_Enable modeSwitchEn,
            Preamble_Inversion preambleInversion,
            FEC_Scheme fec_sheme,
            Interleaving_Enable interleavingEnable);
            
        etl::expected<void, Error> setBbcFskc3Config(
            Transceiver transceiver,
            SFD_Detection_Threshold sfdDetectionThreshold,
            Preamble_Detection_Threshold preambleDetectionThreshold);
                                  
        etl::expected<void, Error> setBbcFskc4Config(
            Transceiver transceiver,
            SFD_Quantization sfdQuantization,
            SFD_32 sfd32,
            Raw_Mode_Reversal_Bit rawModeReversal,
            CSFD1 csfd1,
            CSFD0 csfd0);

        etl::expected<void, Error> setBbcFskphrtx(
            Transceiver transceiver,
            SFD_Used sfdUsed,
            Data_Whitening dataWhitening);

        etl::expected<void, Error> setBbcFskdm(
            Transceiver transceiver,
            FSK_Preamphasis_Enable fskPreamphasisEnable,
            Direct_Mod_Enable_FSKDM directModEnableFskdm);

        etl::expected<void, Error> setExternalFrontEndControl(
            Transceiver transceiver,
            ExternalFrontEndControl frontEndControl);

        etl::expected<uint16_t, Error> getReceivedLength(Transceiver transceiver);

        /**
         * Sets up the target registers. It accesses *all* writable registers and
         * therefore, it requires the transceiver to be in the `TXPREP` state.
         */
        etl::expected<void, Error> setup();
    };


    extern At86rf215_Utilities transceiverUtils;
} // namespace AT86RF215

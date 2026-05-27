/**
 * @file at86rf215.hpp
 *
 * @brief This file contains the functions to interface with the transceiver.
 */

#pragma once

#include "stm32h7xx_hal.h"
#include "etl/optional.h"
#include "etl/delegate.h"
#include "etl/span.h"
#include "etl/expected.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "Logger.hpp"
#include "at86rf215Definitions.hpp"
#include "at86rf215Config.hpp"

namespace AT86RF215 {
    struct IrqStatus {
        etl::optional<uint8_t> rf09IrqsStatus;
        etl::optional<uint8_t> rf24IrqsStatus;
        etl::optional<uint8_t> bbc0IrqsStatus;
        etl::optional<uint8_t> bbc1IrqsStatus;

        IrqStatus() : rf09IrqsStatus(0), rf24IrqsStatus(0), bbc0IrqsStatus(0), bbc1IrqsStatus(0) {};
    };

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
        BASEBAND_OPERATION_FUNCTION_FAILED,
        FAILED_DUE_TO_DESYNCHRONIZATION,
        EVENT_WAIT_TIMEOUT,
        BASEBAND_CORE_DISABLED,
        NO_ERROR
    };

    inline uint8_t operator&(const uint8_t a, InterruptMask b) {
        return a & static_cast<uint8_t>(b);
    }

    typedef struct {
        uint8_t dotDashMapping;  // 0bXX represents the dot-dash mapping (e.g., 0b01 for dot-dash)
        uint8_t dotDashNum;      // The number of symbols in the Morse code
    } MorseCodeMapping;

    /**
     * @brief This class contains methods for operating the transceiver, along with the necessary state variables
     *        required for managing it.
     */
    class AT86RF215Chip {
    public:
        AT86RF215Chip() = default;

        /**
         * @brief Event group for signaling various events. Look in "at86rf215definitions" for interpretation
         *        of each group bit
         */
        EventGroupHandle_t eventGroupHandle;

        /**
         * Initializer for AT86RF215 driver. This function must be called prior to performing
         * any operation with the transceiver.
         */
        etl::expected<void, Error> initializeResources(
            SPI_HandleTypeDef* spi_handle,
            GeneralConfiguration&& general_config = GeneralConfiguration::defaultGeneralConfig(),
            RXConfig&& rx_config = RXConfig::defaultRXConfig(),
            TXConfig&& tx_config = TXConfig::defaultTXConfig(),
            BasebandCoreConfig&& baseband_core_config = BasebandCoreConfig::defaultBasebandCoreConfig(),
            FrequencySynthesizerConfig&& frequency_synthesizer_config = FrequencySynthesizerConfig::defaultFrequencySynthesizerConfig(),
            ExternalFrontEndConfig&& external_front_end_config = ExternalFrontEndConfig::defaultExternalFrontEndConfig(),
            BasebandCoreInterruptsConfig&& baseband_core_interrupts_config = BasebandCoreInterruptsConfig::defaultBasebandCoreInterruptsConfig(),
            RadioInterruptsConfig&& radio_interrupts_config = RadioInterruptsConfig::defaultRadioInterruptsConfig(),
            IQInterfaceConfig&& iq_interface_config = IQInterfaceConfig::defaultIQInterfaceConfig());

        /**
         * This method reads the transceiver interrupt code and takes any necessary actions.
         * It should be used inside a high priority freertos task, dedicated solely to transceiver irq handling.
         *
         * @returns A struct with the status of the interrupt registers (RF09_IRQS, RF24_IRQS, BBC0_IRQS, BBC1_IRQS),
         *          as well as whether an error occurred
         *
         * @note In case an error occurs while reading an interrupt register, the function immediately returns, setting
         *       it's value (and any subsequent registers that have not been read yet) to 0.
         *
         */
        etl::pair<Error, IrqStatus> handleIrq();

        /**
         * Update the configuration structures.
         * @warning For the changes to apply, a subsequent call to chipReset() is required.
         */
        void setGeneralConfig(GeneralConfiguration&& GeneralConfig) {
            generalConfig = GeneralConfig;
        }
        void setRXConfig(RXConfig&& RXConfig) {
            rxConfig = RXConfig; // Move the new config into rxConfig
        }
        void setTXConfig(TXConfig&& TXConfig) {
            txConfig = TXConfig; // Move the new config into rxConfig
        }
        void setBaseBandCoreConfig(BasebandCoreConfig&& BasebandCoreConfig) {
            basebandCoreConfig = BasebandCoreConfig; // Move the new config into rxConfig
        }
        void setFrequencySynthesizerConfig(FrequencySynthesizerConfig&& FrequencySynthesizer) {
            freqSynthesizerConfig = FrequencySynthesizer; // Move the new config into rxConfig
        }
        void setExternalFrontEndControlConfig(ExternalFrontEndConfig&& ExternalFrontEndConfig) {
            externalFrontEndConfig = ExternalFrontEndConfig;
        }
        void setBasebandInterruptConfig(BasebandCoreInterruptsConfig&& InterruptsConfig) {
            basebandCoreInterruptsConfig = InterruptsConfig;
        }
        void setRadioInterruptConfig(RadioInterruptsConfig&& RadioInterruptsConfig) {
            radioInterruptsConfig = RadioInterruptsConfig;
        }
        void setIQInterfaceConfig(IQInterfaceConfig&& IQInterfaceConfig) {
            iqInterfaceConfig = IQInterfaceConfig;
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
         * It also restores the default config settings
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
         * Set the chip to deep sleep mode, in order to minimize current consumption.
         *
         * @note The register settings are lost when the transceiver is set to DEEP_SLEEP.
         */
        etl::expected<void, Error> setDeepSleep();

        /**
         * Wake the transceiver from deep sleep.
         *
         * @note This should be used as a debugging function. In nominal operations, the user
         *       should call chip reset instead, in order to re-configure the transceiver settings
         */
        etl::expected<void, Error> wakeFromDeepSleep();

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
         * Transmit a packet using the internal baseband core (basic mode, without embedded MAC functionality).
         *
         * @param transceiver		Specifies the transceiver used
         * @param packet			The packet data. The size must be smaller than the maximum packet length, which is
         *                          2047
         */
        etl::expected<void, Error> packetTransmissionBaseband(Transceiver transceiver, etl::span<uint8_t> packet);

        /**
         * Set the receiver to a "listening" state, so that packet reception through the
         * baseband core may be performed (basic mode, without embedded MAC functionality).
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
         * @returns A pair with the received packet length (16 bits) and the contents of the register
         *          BBCn_FSKPHRRX (header information). In case of an error, both members are 0.
         */
        etl::expected<etl::pair<uint16_t, uint8_t>, Error> waitForPacketReceptionBaseband(Transceiver transceiver, uint32_t timeoutDelayMs);

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
        etl::expected<void, Error> packetTransmissionIQEmbeddedControl(Transceiver transceiver, etl::delegate<bool()> basebandOp);

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
        etl::expected<void, Error> transmitMorseCodeOOK(
            Transceiver transceiver,
            float wpm,
            etl::string_view sequence);

        /**
         * Writes a byte to a specified address
         *
         * @param address	Specifies the address to write to
         * @param value		The value to write to the specified address
         *
         * @warning This function is exposed publicly for debugging reasons. It does not offer concurrency protection
         */
        etl::expected<void, Error> spiWrite8(RegisterAddress address, uint8_t value);

        /**
         * Reads a byte to a specified address
         *
         * @param address	Specifies the address to read from
         * @returns 		Returns the read byte
         *
         * @warning This function is exposed publicly for debugging reasons. It does not offer concurrency protection
         */
        etl::expected<uint8_t, Error> spiRead8(RegisterAddress address);

    private:
        /**
         * Mutex for protecting against concurrent access to spi and radio resources
         */
        StaticSemaphore_t spiAccessMutexBuffer = {};
        SemaphoreHandle_t spiAccessMutexHandle;

        StaticSemaphore_t transceiver09MutexBuffer = {};
        SemaphoreHandle_t transceiver09MutexHandle;

        StaticSemaphore_t transceiver24MutexBuffer = {};
        SemaphoreHandle_t transceiver24MutexHandle;

        StaticSemaphore_t iqTxMutexBuffer = {};
        SemaphoreHandle_t iqTxMutexHandle;

        /**
         * Event group for signaling various events
         */
        StaticEventGroup_t eventGroupBuffer;

        /**
         * SPI handle
         */
        SPI_HandleTypeDef* hspi;

        /**
         * Structs with register configurations
         */
        GeneralConfiguration generalConfig;
        RXConfig rxConfig;
        TXConfig txConfig;
        BasebandCoreConfig basebandCoreConfig;
        FrequencySynthesizerConfig freqSynthesizerConfig;
        ExternalFrontEndConfig externalFrontEndConfig;
        BasebandCoreInterruptsConfig basebandCoreInterruptsConfig;
        RadioInterruptsConfig radioInterruptsConfig;
        IQInterfaceConfig iqInterfaceConfig;

        /**
         * User provided memory for storing a received packet in baseband core operation
         */
        etl::span<uint8_t> destBuffer09;
        etl::span<uint8_t> destBuffer24;

        /**
         * Received packet's length and header contents in baseband core operation
         */
        uint16_t receivedPacketLength09;
        uint16_t receivedPacketLength24;
        uint8_t bbc09_fskphrrx;
        uint8_t bbc24_fskphrrx;

        /**
         * RAII like objects for mutex locking and mode setup management. Look at @file at86rf215GuardUtilities.hpp
         *
         * @note The classes are declared here, so that they have access to all of AT86RF215Chip's state
         */
        class MutexGuard;
        class DacOverrideSetup;
        class SingleShotMeasurementSetup;
        class IntBasebandCoreBasicModeSetup;

        /**
         * If a frame reception begins with the baseband core, (ReceiverFrameStart interrupt arrives), handleIrq() sets
         * the basebandCoreIsReceiving_n flag, and stores the start time in the basebandCoreReceptionStartTime_n
         * variable. The mutex handler performs checks and refuses to lock a transceiver mutex, if a reception is taking
         * place, unless of course the total amount of reception time has expired (which would indicate something went
         * wrong with the reception. In that scenario, the basebandCoreIsReceiving_n flag is reset. Since the check
         * of these variables is very fast, their access is protected with taskENTER_CRITICAL()
         */
        volatile bool basebandCoreIsReceiving09;
        volatile TickType_t basebandCoreReceptionStartTime09;
        volatile bool basebandCoreIsReceiving24;
        volatile TickType_t basebandCoreReceptionStartTime24;

        /**
         * Yield the spi mutex and wait for the interrupt handling task to process and notify about an event
         */
        etl::expected<void, Error> waitForIrqEvent(
            MutexGuard& mutexGuard,
            IrqEventGroupBit irqEventGroupBit,
            uint16_t waitDelayMs);

        /**
         * If the setup object destructors fail to revert the transceiver to the original configuration, then a
         * desynchronization bit is raised. Every public function must call this synchronizeConfig() to fix the
         * configuration if required.
         *
         * @returns Whether a synchronization was performed or not
         */
        etl::expected<bool, Error> synchronizeConfig() {
            if ((xEventGroupGetBits(eventGroupHandle) & ConfigDesynchronizationGroupBit) != 0) {
                if (auto status = chipReset(); !status.has_value()) {
                    return etl::unexpected(status.error());
                }
                return true;
            }
            return false;
        }

        /**
         * Return the morse code dot-dash mapping of a character
         */
        static constexpr MorseCodeMapping getMorse(char c);

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
         * Overwrite certain bits of the register
         *
         * @param address       Specifies the address to write to
         * @param mask          The bits that will be overwritten
         * @param overwriteBits The new bit values (only the ones where the bitmask has a bit equal to 1 will be applied)
         * @return The original register value
         */
        etl::expected<uint8_t, Error> spiOverwriteBits(
            RegisterAddress address,
            uint8_t mask,
            uint8_t overwriteBits);

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
            DirectModEnableFSKDM directMod,
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
            BandwidthTimeProduct bt,
            ModIndexScale midxs,
            ModIndex midx,
            FskModOrder mord);

        etl::expected<void, Error> setBbcFskc1Config(
            Transceiver transceiver,
            FreqInversion freqInv,
            MrFskSymbolRate sr);

        etl::expected<void, Error> setBbcFskc2Config(
            Transceiver transceiver,
            PreambleDetection preambleDet,
            ReceiverOverride recOverride,
            ReceiverPreambleTimeout recPreambleTimeout,
            ModeSwitchEnable modeSwitchEn,
            PreambleInversion preambleInversion,
            FecScheme fec_sheme,
            InterleavingEnable interleavingEnable);
            
        etl::expected<void, Error> setBbcFskc3Config(
            Transceiver transceiver,
            SfdDetectionThreshold sfdDetectionThreshold,
            PreambleDetectionThreshold preambleDetectionThreshold);
                                  
        etl::expected<void, Error> setBbcFskc4Config(
            Transceiver transceiver,
            SfdQuantization sfdQuantization,
            Sfd32 sfd32,
            RawModeReversalBit rawModeReversal,
            CSFD1 csfd1,
            CSFD0 csfd0);

        etl::expected<void, Error> setBbcFskphrtx(
            Transceiver transceiver,
            SfdUsed sfdUsed,
            DataWhitening dataWhitening);

        etl::expected<void, Error> setBbcFskdm(
            Transceiver transceiver,
            FskPreamphasisEnable fskPreamphasisEnable,
            DirectModEnableFSKDM directModEnableFskdm);

        etl::expected<void, Error> setFskPreambleLength(Transceiver transceiver, uint16_t preambleLength);

        etl::expected<void, Error> setExternalFrontEndControl(
            Transceiver transceiver,
            ExternalFrontEndControl frontEndControl);

        etl::expected<void, Error> setTxDaci(Transceiver transceiver, bool inputEnable, uint8_t input);

        etl::expected<void, Error> setTxDacq(Transceiver transceiver, bool inputEnable, uint8_t input);

        etl::expected<uint16_t, Error> getReceivedLength(Transceiver transceiver);

        /**
         * Sets up the target registers. It accesses *all* writable registers and
         * therefore, it requires the transceiver to be in the `TXPREP` state.
         */
        etl::expected<void, Error> setup();
    };

    extern AT86RF215Chip at86rf215Chip;
} // namespace AT86RF215

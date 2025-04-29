#pragma once

#include <utility>
#include <cstdint>
#include "etl/expected.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Logger.hpp"
#include "at86rf215definitions.hpp"
#include "at86rf215config.hpp"

typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;

namespace AT86RF215 {
    typedef struct {
        uint8_t dotDashMapping;  // 0bXX represents the dot-dash mapping (e.g., 0b01 for dot-dash)
        uint8_t dotDashNum;      // The number of symbols in the Morse code
    } MorseCodeMapping;

    static constexpr MorseCodeMapping getMorse(char c);

    enum class Error {
        NO_ERRORS,
        FAILED_WRITING_TO_REGISTER,
        FAILED_READING_FROM_REGISTER,
        FAILED_CHANGING_STATE,
        UKNOWN_REQUESTED_STATE,
        UKNOWN_PART_NUMBER,
        INVALID_TRANSCEIVER_FREQ,
        INVALID_STATE_FOR_OPERATION,
        INVALID_PLL_CENTER_FREQ,
        UKNOWN_DEVICE_PART_NUMBER,
        INVALID_RSSI_MEASUREMENT,
        INVALID_AGC_CONTROl_WORD,
        ONGOING_TRANSMISSION_RECEPTION,
        RESOURCE_MUTEX_TIMEOUT,
        TRANSMISSION_FAILED,
        RECEPTION_FAILED,
        SINGLE_SHOT_ENERGY_MEASUREMENT_FAILED,
    };

    inline uint8_t operator&(const uint8_t a, InterruptMask b) {
        return a & static_cast<uint8_t>(b);
    }

    class At86rf215_Utilities {
    public:
        /// Flags indicating a radio interrupt has occurred (offered for debugging purposes only, must be manually reset)
        bool IFSynchronization_flag = false;
        bool TransceiverError_flag = false;
        bool EnergyDetectionCompletion_flag = false;
        bool TransceiverReady_flag  = false;
        bool Wakeup_flag = false;
        bool BatteryLow_flag = false;

        /// Flags indicating a baseband core interrupt has occurred (offered for debugging purposes only, must be manually reset)
        bool FrameBufferLevelIndication_flag = false;
        bool AGCRelease_flag = false;
        bool AGCHold_flag = false;
        bool TransmitterFrameEnd_flag = false;
        bool ReceiverExtendMatch_flag = false;
        bool ReceiverAddressMatch_flag = false;
        bool ReceiverFrameEnd_flag = false;
        bool ReceiverFrameStart_flag = false;

        /// Binary semaphores for signaling certain events (add them inside the proper ISR or freertos tak)
        SemaphoreHandle_t spiWriteCompleteSemaphoreHandle;          // completion of spi write from dma callback
        SemaphoreHandle_t spiReadCompleteSemaphoreHandle;           // completion of spi read from dma callback
        SemaphoreHandle_t iqEecTransmissionCompleteSemaphoreHandle09; // completion of tx using I/Q interface with embedded control
        SemaphoreHandle_t iqPreambleReceptionSemaphoreHandle09;       // reception of a preamble using the I/Q interface
        SemaphoreHandle_t iqPacketReceptionSemaphoreHandle09;         // full reception of a packet using the I/Q interface
        SemaphoreHandle_t iqEecTransmissionCompleteSemaphoreHandle24; // completion of tx using I/Q interface with embedded control
        SemaphoreHandle_t iqPreambleReceptionSemaphoreHandle24;       // reception of a preamble using the I/Q interface
        SemaphoreHandle_t iqPacketReceptionSemaphoreHandle24;         // full reception of a packet using the I/Q interface

        /**
         * Initializer for AT86RF215 driver
         */
        At86rf215_Utilities()
                : transceiverOccupied09(false), transceiverOccupied24(false),
                  userRequest09(UserRequest::NO_REQUEST), userRequest24(UserRequest::NO_REQUEST),
                  energy_measurement09(0), energy_measurement24(0), received_packet_length09(0),
                  received_packet_length24(0) {
            // Initialize the mutex and the binary semaphores
            resourcesMutexHandle = xSemaphoreCreateMutexStatic(&resourcesMutexBuffer);
            spiWriteCompleteSemaphoreHandle = xSemaphoreCreateBinaryStatic(&spiWriteCompleteSemaphoreBuffer);
            spiReadCompleteSemaphoreHandle = xSemaphoreCreateBinaryStatic(&spiReadCompleteSemaphoreBuffer);
            basebandTx09SemaphoreHandle = xSemaphoreCreateBinaryStatic(&basebandTx09SemaphoreBuffer);
            basebandTx24SemaphoreHandle = xSemaphoreCreateBinaryStatic(&basebandTx24SemaphoreBuffer);
            basebandRx09SemaphoreHandle = xSemaphoreCreateBinaryStatic(&basebandRx09SemaphoreBuffer);
            basebandRx24SemaphoreHandle = xSemaphoreCreateBinaryStatic(&basebandRx24SemaphoreBuffer);
            energyDetCompletion09SemaphoreHandle = xSemaphoreCreateBinaryStatic(&energyDetCompletion09SemaphoreBuffer);
            energyDetCompletion24SemaphoreHandle = xSemaphoreCreateBinaryStatic(&energyDetCompletion24SemaphoreBuffer);
            iqEecTransmissionCompleteSemaphoreHandle09 = xSemaphoreCreateBinaryStatic(&iqEecTransmissionCompleteSemaphoreBuffer09);
            iqPreambleReceptionSemaphoreHandle09 = xSemaphoreCreateBinaryStatic(&iqPreambleReceptionSemaphoreBuffer09);
            iqPacketReceptionSemaphoreHandle09 = xSemaphoreCreateBinaryStatic(&iqPacketReceptionSemaphoreBuffer09);
            iqEecTransmissionCompleteSemaphoreHandle24 = xSemaphoreCreateBinaryStatic(&iqEecTransmissionCompleteSemaphoreBuffer24);
            iqPreambleReceptionSemaphoreHandle24 = xSemaphoreCreateBinaryStatic(&iqPreambleReceptionSemaphoreBuffer24);
            iqPacketReceptionSemaphoreHandle24 = xSemaphoreCreateBinaryStatic(&iqPacketReceptionSemaphoreBuffer24);
            if (resourcesMutexHandle == nullptr ||
                basebandTx09SemaphoreHandle == nullptr ||
                basebandTx24SemaphoreHandle == nullptr ||
                basebandRx09SemaphoreHandle == nullptr ||
                basebandRx24SemaphoreHandle == nullptr ||
                energyDetCompletion09SemaphoreHandle == nullptr ||
                energyDetCompletion24SemaphoreHandle == nullptr ||
                spiWriteCompleteSemaphoreHandle == nullptr ||
                spiReadCompleteSemaphoreHandle == nullptr ||
                iqEecTransmissionCompleteSemaphoreHandle09 == nullptr ||
                iqPreambleReceptionSemaphoreHandle09 == nullptr ||
                iqPacketReceptionSemaphoreHandle09 == nullptr ||
                iqEecTransmissionCompleteSemaphoreHandle24 == nullptr ||
                iqPreambleReceptionSemaphoreHandle24 == nullptr ||
                iqPacketReceptionSemaphoreHandle24 == nullptr) {
                LOG_ERROR << "[AT86RF215 Driver] Failed to create semaphores";
            }

            // Set the default configuration structures
            setGeneralConfig();
            setRXConfig();
            setTXConfig();
            setBaseBandCoreConfig();
            setFrequencySynthesizerConfig();
            setExternalFrontEndControlConfig();
            setInterruptConfig();
            setRadioInterruptConfig();
            setIQInterfaceConfig();
        }

        /**
         * Register SPI handle, and setup the transceiver.
         * @warning The user should always call this before using any of the driver methods.
         */
        void registerTransceiver(SPI_HandleTypeDef* handle, Error err = Error::NO_ERRORS) {
            hspi = handle;
            setup(err);
        }

        /**
         * This method reads the transceiver interrupt code and takes any necessary actions.
         * It should be used inside a high priority freertos task, dedicated solely to transceiver irq handling.
         *
         * @warning This method attempts to take the resources mutex, so it must not be called inside an ISR. Instead,
         *          the ISR should notify the dedicated irq handling task.
         */
        void handle_irq(Error &err);

        /**
         * Update the configuration structures. For the changes to apply, a subsequent call to chip_reset() is
         * required.
         */
        void setGeneralConfig(GeneralConfiguration&& GeneralConfig = GeneralConfiguration::DefaultGeneralConfig()) {
            generalConfig = std::move(GeneralConfig);
        }
        void setRXConfig(RXConfig&& RXConfig = RXConfig::DefaultRXConfig()) {
            rxConfig = std::move(RXConfig); // Move the new config into rxConfig
        }
        void setTXConfig(TXConfig&& TXConfig = TXConfig::DefaultTXConfig()) {
            txConfig = std::move(TXConfig); // Move the new config into rxConfig
        }
        void setBaseBandCoreConfig(BasebandCoreConfig&& BasebandCoreConfig = BasebandCoreConfig::DefaultBasebandCoreConfig()) {
            basebandCoreConfig = std::move(BasebandCoreConfig); // Move the new config into rxConfig
        }
        void setFrequencySynthesizerConfig(FrequencySynthesizerConfig&& FrequencySynthesizer = FrequencySynthesizerConfig::DefaultFrequencySynthesizerConfig()) {
            freqSynthesizerConfig = std::move(FrequencySynthesizer); // Move the new config into rxConfig
        }
        void setExternalFrontEndControlConfig(ExternalFrontEndConfig&& ExternalFrontEndConfig = ExternalFrontEndConfig::DefaultExternalFrontEndConfig()) {
            externalFrontEndConfig = std::move(ExternalFrontEndConfig);
        }
        void setInterruptConfig(BasebandCoreInterruptsConfig&& InterruptsConfig = BasebandCoreInterruptsConfig::DefaultBasebandCoreInterruptsConfig()) {
            basebandCoreInterruptsConfig = std::move(InterruptsConfig);
        }
        void setRadioInterruptConfig(RadioInterruptsConfig&& RadioInterruptsConfig = RadioInterruptsConfig::DefaultRadioInterruptsConfig()) {
            radioInterruptsConfig = std::move(RadioInterruptsConfig);
        }
        void setIQInterfaceConfig(IQInterfaceConfig&& IQInterfaceConfig = IQInterfaceConfig::DefaultIQInterfaceConfig()) {
            iqInterfaceConfig = std::move(IQInterfaceConfig);
        }

        /**
         * Fetches the current state of the transceiver
         * @note Mutex protected wrapper for get_state_private()
         *
         * @param transceiver	Specifies the transceiver used
         * @param err			Pointer to raised error
         */
        State get_state(Transceiver transceiver, Error& err);

        /**
         * Sets the state of the transceiver
         * @note Mutex protected wrapper for set_state_private()
         *
         * @param transceiver	Specifies the transceiver used
         * @param state_cmd		Command responsible for changing the state
         * @param err			Pointer to raised error
         */
        void set_state(Transceiver transceiver, State state_cmd, Error& err);

        /**
         * Does chip reset and reads from the interrupt status registers via SPI, resetting them.
         * It also restores the config settings
         * @param error		Pointer to raised error
         */
        void chip_reset(Error& error);

        /**
         * Try to read something from the transceiver to ensure the spi connection works
         */
        etl::expected<void, Error> check_transceiver_connection(Error& err);

        /**
         * Use the logger to print the current state of the transceiver
         */
        void print_state(Transceiver transceiver, Error& err);

        /**
         * Print an error using the logger
         */
        void print_error(Error& err);

        /**
         * Begin operations for measuring energy in the specified bandwidth (single shot measurement)
         * @param transceiver       Selected transceiver
         * @param err               Pointer to raised error
         */
        // TODO: specify bw here
        int8_t clear_channel_assessment(Transceiver transceiver, Error& err);

        /**
         * Transmit a packet using the baseband core.
         *
         * @param transceiver		Specifies the transceiver used
         * @param packet			Pointer to packet data
         * @param length			Length of packet
         * @param err				Pointer to raised error
         *
         */
        void packetTransmissionBaseband(Transceiver transceiver, uint8_t* packet,
                                        uint16_t length, Error& err);

        /**
         * Set the receiver to a "listening" state, so that packet reception through the
         * baseband core may be performed.
         *
         * @note This function essentially sets the transceiver to state RX, but the user is
         *       not stopped from performing an energy measurement, or a tx operation (either with
         *       the baseband core or through the I/Q interface), meaning the
         *       transceiverOccupied flag is not set until an actual reception occurs.  The function
         *       has to be called again to re-enter the "listening" state.
         *
         *
         * @param transceiver		Specifies the transceiver used
         * @param destBuff          A user provided buffer to write the packet. In order to guarantee
         *                          that there will be no buffer overflow, it's capacity should be
         *                          at least 2047 (the maximum possible packet length)
         * @param err				Pointer to raised error
         */
        void preparePacketReceptionBaseband(Transceiver transceiver, uint8_t* destBuff, Error &err);

        /**
         * Waits for packet reception. The packet is written to the registered buffer from
         * the preparePacketReceptionBaseband() call.
         *
         * @note The actual copying of the reception packet happens in handle_irq(), when a receiver frame
         *       end interrupt arrives. All this function does is wait for a semaphore,
         *       which is sent when said copying is finished.
         *
         * @returns The received packet length
         */
        uint16_t waitForPacketReceptionBaseband(Transceiver transceiver, Error &err);

        /**
         * Set the transceiver to state TX_PREP and set the transceiverOccupied flag, so that
         * transmission from an external baseband processor may begin.
         *
         * @note This function should be called only when embedded control is active. In this mode,
         *       the transceiver is automatically  set to state TX, by the external baseband processor.
         *       This is achieved by sending I_DATA[0] == 1 through the I/Q interface (@see figure 7.6 of datasheet).
         *
         * @b The user needs to give the iqEecTransmissionCompleteSemaphore immediately after the
         *    baseband processor finishes the TX operation, so that the transceiver occupied flag is reset.
         */
        void packetTransmissionIQEmbeddedControl(Transceiver transceiver, Error &err);

        /**
         * Set the transceiver to a "listening" state , so that packet reception through the
         * I/Q interface may be performed.
         *
         * @note This function essentially sets the transceiver to state RX, but the user is
         *       not stopped from performing an energy measurement, or a tx operation (either with
         *       the baseband core or through the I/Q interface), meaning the
         *       transceiverOccupied flag is not set until an actual reception occurs. The function
         *       has to be called again to re-enter the "listening" state.
         *
         */
        void preparePacketReceptionIQ(Transceiver transceiver, Error& err);

        /**
         * Wait for packet reception through the I/Q interface.
         *
         * @note The user needs to take the following actions externally:
         *    - give the iqPreambleReceptionSemaphore immediately after the external baseband processor
         *      detects a preamble, so that the transceiver is locked (transceiverOccupied flag set) and the
         *      AGC frozen.
         *
         *    - give the iqPacketReceptionSemaphore once the external baseband processor fully received the
         *      packet, so that the AGC is released and the transceiverOccupied flag is reset
         */
        void waitForPacketReceptionIQ(Transceiver transceiver, Error& err);

        /**
         * Transmit a sequence of characters encoded as morse code, with on-off keying modulation (OOK).
         * This is achieved using the "DAC overwrite"  features (section 13.1.2), which allows transmission
         * of a pure LO carrier.
         * @note Ensure IQIFC1.CHPM = 0 and PC.CTX = 1.
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
        void transmitMorseCode(Transceiver transceiver, Error& err, float wpm, const char* sequence, uint16_t sequenceLen);

    private:
        /// Mutex for concurrent access protection
        StaticSemaphore_t resourcesMutexBuffer = {};
        SemaphoreHandle_t resourcesMutexHandle;
        uint16_t mutexTimeout = 100; // in ms
        // TODO maybe it would be better to use a separate timeout for "time critical"
        //      procedures (like freezing the AGC) and a less strict one for stuff like reading the drivers parameters

        /// Binary semaphores for signaling external events
        StaticSemaphore_t spiWriteCompleteSemaphoreBuffer = {};
        StaticSemaphore_t spiReadCompleteSemaphoreBuffer = {};
        StaticSemaphore_t iqEecTransmissionCompleteSemaphoreBuffer09 = {};
        StaticSemaphore_t iqPreambleReceptionSemaphoreBuffer09 = {};
        StaticSemaphore_t iqPacketReceptionSemaphoreBuffer09 = {};
        StaticSemaphore_t iqEecTransmissionCompleteSemaphoreBuffer24 = {};
        StaticSemaphore_t iqPreambleReceptionSemaphoreBuffer24 = {};
        StaticSemaphore_t iqPacketReceptionSemaphoreBuffer24 = {};

        /// Binary semaphores for signaling events from handle_irq()
        StaticSemaphore_t basebandTx09SemaphoreBuffer = {};
        SemaphoreHandle_t basebandTx09SemaphoreHandle;  // signal finished transmission for sub GHz baseband core

        StaticSemaphore_t basebandTx24SemaphoreBuffer = {};
        SemaphoreHandle_t basebandTx24SemaphoreHandle; // signal finished transmission for 2.4 baseband core

        StaticSemaphore_t basebandRx09SemaphoreBuffer = {};
        SemaphoreHandle_t basebandRx09SemaphoreHandle; // signal finished reception for sub GHz baseband core

        StaticSemaphore_t basebandRx24SemaphoreBuffer = {};
        SemaphoreHandle_t basebandRx24SemaphoreHandle; // signal finished reception for 2.4 GHz baseband core

        StaticSemaphore_t energyDetCompletion09SemaphoreBuffer = {};
        SemaphoreHandle_t energyDetCompletion09SemaphoreHandle;

        StaticSemaphore_t energyDetCompletion24SemaphoreBuffer = {};
        SemaphoreHandle_t energyDetCompletion24SemaphoreHandle;

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

        enum class UserRequest {
            BASEBAND_RX,                          // rx with baseband core
            BASEBAND_TX,                          // tx with baseband core
            IQ_EEC_TX,                            // tx with I/Q interface, using embedded control
            IQ_RX,                                // rx with I/Q interface
            SINGLE_SHOT_ENERGY_MEASUREMENT,       // use frontend to measure energy
            NO_REQUEST
        };
        UserRequest userRequest09;
        UserRequest userRequest24;

        /// Indicate whether the radio (and possibly the baseband core) are occupied with a tx/rx/energy
        /// measurement operation
        bool transceiverOccupied09;
        bool transceiverOccupied24;

        /// User provided buffer for storing a received packet in baseband core operation
        uint8_t* destBuffer09;
        uint8_t* destBuffer24;

        /// Received packet's length in baseband core operation
        uint16_t received_packet_length09;
        uint16_t received_packet_length24;

        /// Result of a clear channel assessment is stored here
        int8_t energy_measurement09;
        int8_t energy_measurement24;

        /**
         * Writes a byte to a specified address
         *
         * @param address	Specifies the address to write to
         * @param value		The value to write to the specified address
         * @param err		Pointer to raised error
         */
        void spi_write_8(uint16_t address, uint8_t value, Error& err);

        /**
         * Reads a byte to a specified address
         *
         * @param address	Specifies the address to read from
         * @param err		Pointer to raised error
         * @returns 		Returns the read byte
         */
        uint8_t spi_read_8(uint16_t address, Error& err);

        /**
         * Writes a byte to a specified address
         *
         * @param address	Specifies the address to start writing to
         * @param n			Number of bytes to write
         * @param value		Pointer to array of values to write to address
         * @param err		Pointer to raised error
         */
        void spi_block_write_8(uint16_t address, uint16_t n, uint8_t* value,
                               Error& err);

        /**
         * Reads a byte to a specified address. Assumes that the caller has
         * allocated the expected memory.
         *
         * @param address	Specifies the address to start reading from
         * @param n 		Number of bytes to read.
         * @param response	Returns a pointer to the read bytes
         * @param err		Pointer to raised error
         */
        uint8_t* spi_block_read_8(uint16_t address, uint8_t n, uint8_t* response,
                                  Error& err);

        /**
         * Fetches the current state of the transceiver
         *
         * @param transceiver	Specifies the transceiver used
         * @param err			Pointer to raised error
         */
        State get_state_private(Transceiver transceiver, Error& err);

        /**
         * Sets the state of the transceiver
         *
         * @param transceiver	Specifies the transceiver used
         * @param state_cmd		Command responsible for changing the state
         * @param err			Pointer to raised error
         */
        void set_state_private(Transceiver transceiver, State state_cmd, Error& err);

        /**
         * Sets PLL channel spacing (25kHz resolution)
         *
         * @param transceiver	Specifies the transceiver used
         * @param spacing	Configures the channel spacing with a resolution of 25kHz
         * @param err		Pointer to raised error
         */
        void set_pll_channel_spacing(Transceiver transceiver, uint8_t spacing,
                                     Error& err);

        /**
         * Gets PLL channel spacing
         * @param transceiver	Specifies the transceiver used
         * @param err		Pointer to raised error
         */
        uint8_t get_pll_channel_spacing(Transceiver transceiver, Error& err);

        /**
         * Sets the central channel frequency of the PLL
         *
         * @param transceiver	Specifier the transceiver used
         * @param freq 			Central frequency of the PLL
         * @param err			Pointer to raised error
         */
        void set_pll_channel_frequency(Transceiver transceiver, uint16_t freq,
                                       Error& err);

        /**
         * Fetches the central channel frequency of the PLL
         *
         * @param transceiver	Specifier the transceiver used
         * @param err			Pointer to raised error
         */
        uint16_t get_pll_channel_frequency(Transceiver transceiver, Error& err);

        /**
         * Gets the channel number of the PLL
         *
         * @param transceiver	Specifier the transceiver used
         * @param err			Pointer to raised error
         */
        uint16_t get_pll_channel_number(Transceiver transceiver, Error& err);

        /**
         * Sets the loop bandwitdh of the PLL. Options are:
         * 	- Default (0x0)
         * 	- 15% smaller than default (0x1)
         * 	- 15% larger than default (0x2)
         * 	This is only applicable to the RF09 transceiver
         *
         * @param bw	Loopbandwidth of PLL
         * @param err	Pointer to raised error
         */
        void set_pll_bw(PLLBandwidth bw, Error& err);

        /**
         * Gets the loop bandwitdh of the PLL. Options are:
         * 	- Default
         * 	- 15% smaller than default
         * 	- 15% larger than default
         * 	This is only applicable to the RF09 transceiver
         *
         * @param err	Pointer to raised error
         * @returns 	PLL bandwidth
         */
        PLLBandwidth get_pll_bw(Error& err);

        /**
         * Gets the state of the PLL (locked/not locked)
         *
         * @param transceiver		Specify the transceiver used
         * @param err				Pointer to raised error
         */
        PLLState get_pll_state(Transceiver transceiver, Error& err);

        /**
         * Configures the PLL
         *
         * @param transceiver		         Specify the transceiver used
         * @param frequencySynthesizerConfig Reference to configuration with frequency, channel mode and bandwidth
         * @param err				         Pointer to raised error
         */
        void configure_pll(Transceiver transceiver, FrequencySynthesizerConfig& frequencySynthesizerConfig, Error& err);

        /**
         * Gets the part number of the device
         *
         * @param err	Pointer to raised error
         * @returns 	The part number that is one of the following:
         * 					- AT86RF215
         * 					- AT86RF215IQ
         * 					- AT86RF215M
         */
        DevicePartNumber get_part_number(Error& err);

        /**
         * Gets the version number of the device
         *
         * @param err	Pointer to raised error
         */
        DeviceVersionNumber get_version_number(Error& err);

        /**
         * Sets the PLL frequency
         *
         * @param transceiver	Specify the transceiver used
         * @param freq			PLL frequency
         * @param err			Pointer to raised error
         */
        void set_pll_frequency(Transceiver transceiver, uint8_t freq, Error& err);

        /**
         * Gets the PLL frequency
         *
         * @param transceiver	Specify the transceiver used
         * @param err			Pointer to raised error
         * @return 				PLL frequency
         */
        uint8_t get_pll_frequency(Transceiver transceiver, Error& err);

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
         * @param err	Pointer to raised error
         */
        void set_tcxo_trimming(CrystalTrim trim, Error& err);

        /**
         * Reads trimming capacitor to match the load of external TXCO (if used), with
         * a precision of 0.3 pF.
         *
         * @param err	Pointer to raised error
         */
        CrystalTrim read_tcxo_trimming(Error& err);

        /**
         * Set fast start-up enable option for external crystal oscillator
         * If enabled, it will increase start-up time by 0.8mA while also increasing
         * the start-up time.
         *
         * @param fast_start_up		Fast start-up option for TCXO
         * @param err				Pointer to raised error
         */
        void set_tcxo_fast_start_up_enable(bool fast_start_up, Error& err);

        /**
         * Reads fast start-up enable option for external crystal oscillator
         * If enabled, it will increase start-up time by 0.8mA while also increasing
         * the start-up time.
         *
         * @param err				Pointer to raised error
         */
        bool read_tcxo_fast_start_up_enable(Error& err);

        /**
         * Set PA ramp-up time in TX chain.
         *
         * Longer ramp-up time requires more power but decreases possible spurious emissions
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return 					PA ramp-up time
         */
        PowerAmplifierRampTime get_pa_ramp_up_time(Transceiver transceiver,
                                                   Error& err);
        /**
         * Get the low pass cut-off frequency of the filter in the TX chain.
         * For the filter response refer to Figure 6-2, Atmel AT86RF215 datasheet
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return 					Filter cutoff frequency
         */
        TransmitterCutOffFrequency get_cutoff_freq(Transceiver transceiver,
                                                   Error& err);

        /**
         * Get the relative cut-off frequency of the filter in the TX chain.
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return 					Filter cutoff frequency
         */
        TxRelativeCutoffFrequency get_relative_cutoff_freq(Transceiver transceiver,
                                                           Error& err);
        /**
         * Get whether direct modulation is used in the TX chain.
         * Only available for baseband FSK and OQPSK)
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return 					Indicates whether direct modulation is used
         */
        bool get_direct_modulation(Transceiver transceiver, Error& err);

        /**
         * Set the sample rate of the receiver.
         * For exact configuration of the sample_rate refer to AT86RF215 datasheet, Table 6-6
         * or in registers.h*
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return 					Sample rate of receiver
         */
        ReceiverSampleRate get_sample_rate(Transceiver transceiver, Error& err);

        /**
         * Read PA DC current
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return 					PA DC current
         */
        PowerAmplifierCurrentControl get_pa_dc_current(Transceiver transceiver,
                                                       Error& err);

        /**
         * Get whether the external LNA is bypassed
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return					Get whether external LNA is bypassed
         */
        bool get_lna_bypassed(Transceiver transceiver, Error& err);

        /**
         * Shows whether Automatic Gain Control is used for the external LNA.
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return agcmap			AGC gain
         */
        AutomaticGainControlMAP get_agcmap(Transceiver transceiver, Error& err);

        /**
         * Set whether an external analog voltage is supplied to AVDD0 or AVDD1 for the sub-1 GHz
         * and the 2.4 Ghz transceiver respectively
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return					Specifies whether external voltage is supplied to AVDD
         */
        AutomaticVoltageExternal get_external_analog_voltage(
                Transceiver transceiver, Error& err);

        /**
         * Shows whether analog voltage is settled
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return					Specifies whether AV is settled
         */
        bool get_analog_voltage_settled_status(Transceiver transceiver, Error& err);

        /**
         * Fetches supplied voltage of the internal PA
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @return					PA supplied voltage
         */
        PowerAmplifierVoltageControl get_analog_power_amplifier_voltage(
                Transceiver transceiver, Error& err);

        /**
         * Set receiver energy detection average duration given by df*dtb
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         * @param df				Detection factor
         * @param dtb				Detection time scale
         */
        void set_ed_average_detection(Transceiver transceiver, uint8_t df,
                                      EnergyDetectionTimeBasis dtb, Error& err);

        /**
         * Read receiver energy detection average duration given by df*dtb in μs
         *
         * @param transceiver		Specifies the transceiver used
         * @param err				Pointer to raised error
         */
        uint8_t get_ed_average_detection(Transceiver transceiver, Error& err);

        int8_t get_receiver_energy_detection(Transceiver transceiver, Error& err);


        /**
         * Set transceiver battery monitor status
         *
         * @param status			Battery monitor status
         * @param err				Pointer to raised error
         */
        void set_battery_monitor_status(bool status, Error& err);

        /**
         * Get transceiver battery monitor status
         *
         * @param err				Pointer to raised error
         * @return status			Battery monitor status
         */
        BatteryMonitorStatus get_battery_monitor_status(Error& err);

        /**
         * Set the threshold of the battery monitoring range (low/high)
         *
         * @param range				Transceiver battery range
         * @param err				Pointer to raised error
         */
        void set_battery_monitor_high_range(BatteryMonitorHighRange range,
                                            Error& err);

        /**
         * Gets the threshold of the battery monitoring range (low/high)
         *
         * @param err				Pointer to raised error
         * @return range			Transceiver battery monitoring range
         */
        uint8_t get_battery_monitor_high_range(Error& err);

        /**
         * Sets voltage threshold for battery monitoring
         *
         * @param threshold			Battery voltage threshold
         * @param err				Pointer to raised error
         */
        void set_battery_monitor_voltage_threshold(BatteryMonitorVoltageThreshold threshold,
                                                   Error& err);
        void set_battery_monitor_control(BatteryMonitorHighRange range, BatteryMonitorVoltageThreshold threshold, Error& err);

        /**
         * Get voltage threshold for battery monitoring
         *
         * @param err				Pointer to raised error
         * @return threshold		Battery voltage threshold
         */
        uint8_t get_battery_monitor_voltage_threshold(Error& err);

        /**
         * Sets up the target registers for setting up the transceiver tx frontend
         *
         * @param transceiver		Specifies the transceiver used
         * @param pa_ramp_time	    TX PA ramp time
         * @param cutoff 			TX filter cut-off frequency
         * @param tx_rel_cutoff     TX relative cut-off frequency
         * @param direct_mod		Specifies whether direct modulation is supported (supported for FSK and OQPSK)
         * @param tx_sample_rate    TX sample rate
         * @param pa_curr_control 	Controls power amplifier current reduction
         * @param transceiver		Specifies the transceiver used
         * @param tx_out_power		Output power of the transmitter (0x00-0x1F in 1dB steps)
         * @param ext_lna_bypass 	Specifies whether external LNA will be bypassed
         * @param agc_map			Controls gain of the gain controler for the external LNA
         * @param avg_ext			Disables internal supply voltage
         * @param av_enable			Defines whether voltage regulator is enabled during TRXOFF
         * @param pa_vcontrol		Controls supply voltage of internal PA
         * @param err				Pointer to raised error
         */
        void setup_tx_frontend(Transceiver transceiver,
                               PowerAmplifierRampTime pa_ramp_time,
                               TransmitterCutOffFrequency cutoff,
                               TxRelativeCutoffFrequency tx_rel_cutoff, Direct_Mod_Enable_FSKDM direct_mod,
                               TransmitterSampleRate tx_sample_rate,
                               PowerAmplifierCurrentControl pa_curr_control, uint8_t tx_out_power,
                               ExternalLNABypass ext_lna_bypass, AutomaticGainControlMAP agc_map,
                               AutomaticVoltageExternal avg_ext, AnalogVoltageEnable av_enable,
                               PowerAmplifierVoltageControl pa_vcontrol, ExternalFrontEndControl externalFrontEndControl, Error& err);

        /**
         * Sets up the target registers for setting up the transceiver rx frontend
         *
         * @param transceiver		Specifies the transceiver used
         * @param if_inversion		Defines whether IF inverted signal is used in the receive side
         * @param if_shift			If true, it shifts the IF frequency by a factor of 1.25
         * @param rx_bw				Specifies the receiver bandwidth
         * @param rx_rel_cutoff		RX filter relative cut-off frequency
         * @param rx_sample_rate	RX sample rate
         * @param agc_input			If true, the filtered front signal is used rather than the signal before the channel filter
         * @param agc_avg_sample	AGC averaging
         * @param agc_enable 		If set to true AGC is enabled, otherwise, the gain is defined by the agc_gain parameter (AGCS.GCW register)
         * @param agc_target		Sets the target output gain of the AGC
         * @param gain_control_word	If AGC is not enabled, then this register is used to define the maximum gain (valid values 0-23 with 3dB steps)
         * @param err				Pointer to raised error
         */
        void setup_rx_frontend(Transceiver transceiver, bool if_inversion,
                               bool if_shift, ReceiverBandwidth rx_bw,
                               RxRelativeCutoffFrequency rx_rel_cutoff,
                               ReceiverSampleRate rx_sample_rate, bool agc_input,
                               AverageTimeNumberSamples agc_avg_sample, AGCReset agc_reset, AGCFreezeControl agc_freeze_control, AGCEnable agc_enable,
                               AutomaticGainTarget agc_target, uint8_t gain_control_word, Error& err);
        /**
         * Set up IQ interface
         *
         * @param external_loop		Defines whether external loopback is enabled (for testing purposes only)
         * @param out_cur			Defines output current
         * @param common_mode_vol	Voltage of I/Q signals
         * @param common_mode_iee	Whether voltage of I/Q signals is set to 1V2 (IEEE Std 1596-compliant)
         * @param embedded_tx_start	Specifies whether a control bit is automatically transmitted upon start and finish of IQ stream
         * @param chip_mode			Defines what operates out of the baseband core and I/Q IF
         * @param skew_alignment	Specifies the alignment of I/Q data relative to the clock edges of RXCLK
         */
        void setup_iq(ExternalLoopback external_loop, IQOutputCurrent out_cur,
                      IQmodeVoltage common_mode_vol, IQmodeVoltageIEE common_mode_iee,
                      EmbeddedControlTX embedded_tx_start, ChipMode chip_mode,
                      SkewAlignment skew_alignment, Error& err);

        /**
         *  Identify whether the IQ interface deserializer is synchronized
         */
        bool get_iqSyncStatus(Error& err);

        /**
         * Sets up parameters for received energy tracking
         *
         * @param transceiver				Specifies the transceiver used
         * @param energy_mode				Energy detection measurement mode (AUTO/Single/Continuous/Off)
         * @param energy_detect_factor		Duration factor over which the results will be averaged (mult by time base)
         * @param energy_time_basis			Time basis multiplied by the detection factor to determine the averaging window
         * @param err						Pointer to raised error
         */
        void setup_rx_energy_detection(Transceiver transceiver, EnergyDetectionMode energy_mode,
                                       uint8_t energy_detect_factor,
                                       EnergyDetectionTimeBasis energy_time_basis, Error& err);

        /**
         * Sets up internal crystal oscillator
         *
         * @param fast_start_up				Fast start-up option for TCXO (quicker start-up at the expense of current consumption)
         * @param crystal_trim				Controls trim-capacitor to match load capacitance of external oscillator
         * @param err 						Pointer to raised error
         */
        void setup_crystal(bool fast_start_up, CrystalTrim crystal_trim,
                           Error& err);

        /**
         * Sets up IRQ behavior
         *
         * @param maskMode				Defines whether reasons for IRQ call appear in IRQS register
         * @param polarity				Sets up the IRQ pin polarity (active high or low)
         * @param padDriverStrength		Driver strength (mA) of MISO, IRQ and FEA/FEB pins
         * @param err					Pointer to returned error
         */
        void setup_irq_cfg(bool maskMode, IRQPolarity irqPolarity,
                           PadDriverStrength padDriverStrength, Error& err);

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
         * @param err					Pointer to returned error
         */
        void setup_phy_baseband(Transceiver transceiver, bool continuousTransmit, bool frameSeqFilter, bool transmitterAutoFCS,
                                FrameCheckSequenceType fcsType, bool basebandEnable, PhysicalLayerType phyType, Error& err);

        void setup_irq_mask(Transceiver transceiver, bool iqIfSynchronizationFailure, bool transceiverError,
                            bool batteryLow, bool energyDetectionCompletion, bool transceiverReady, bool wakeup,
                            bool frameBufferLevelIndication, bool agcRelease, bool agcHold,
                            bool transmitterFrameEnd, bool receiverExtendedMatch, bool receiverAddressMatch,
                            bool receiverFrameEnd, bool receiverFrameStart, Error& err);

        /**
         *
         * Returns the IRQ register from the corresponding transceiver
         *
         * @param transceiver		Target transceiver
         * @param err				Pointer to raised error
         */
        uint8_t get_irq(Transceiver transceiver, Error& err);

        void set_bbc_fskc0_config(Transceiver transceiver,
                                  Bandwidth_time_product bt, Mod_index_scale midxs, Mod_index midx, FSK_mod_order mord,
                                  Error& err);
        void set_bbc_fskc1_config(Transceiver transceiver,
                                  Freq_Inversion freq_inv, MR_FSK_symbol_rate sr,
                                  Error& err);
        void set_bbc_fskc2_config(Transceiver transceiver, Preamble_Detection preamble_det,
                                  Receiver_Override rec_override,
                                  Receiver_Preamble_Timeout rec_preamble_timeout,
                                  Mode_Switch_Enable mode_switch_en,
                                  Preamble_Inversion preamble_inversion,
                                  FEC_Scheme fec_sheme,
                                  Interleaving_Enable interleaving_enable, Error& err);
        void set_bbc_fskc3_config(Transceiver transceiver, SFD_Detection_Threshold sfdDetectionThreshold,
                                  Preamble_Detection_Threshold preambleDetectionThreshold,
                                  Error& err);
        void set_bbc_fskc4_config(Transceiver transceiver,
                                  SFD_Quantization sfd_quantization,
                                  SFD_32 sfd_32,
                                  Raw_Mode_Reversal_Bit raw_mode_reversal,
                                  CSFD1 csfd1,
                                  CSFD0 csfd0,
                                  Error& err);
        void set_bbc_fskphrtx(Transceiver transceiver,
                              SFD_Used sfdUsed,
                              Data_Whitening dataWhitening,
                              Error& err);
        void set_bbc_fskdm(Transceiver transceiver,
                           FSK_Preamphasis_Enable fskPreamphasisEnable,
                           Direct_Mod_Enable_FSKDM directModEnableFskdm,
                           Error& err);
        void set_external_front_end_control(Transceiver transceiver,
                                            ExternalFrontEndControl frontEndControl,
                                            Error& err);

        etl::expected<uint16_t, Error> get_received_length(Transceiver transceiver, Error& err);

        /**
         *  Reads received packet upon reception of RXFE interrupt
         *
         * @param transceiver
         * @param err
         */
        void packetReceptionBaseband(Transceiver transceiver, Error& err);

        /**
         * Sets up the target registers. It accesses *all* writable registers and
         * therefore, it requires the transceiver to be in the `TXPREP` state.
         *
         * @param err				Pointer to raised error
         */
        void setup(Error& err);
    };


    extern At86rf215_Utilities transceiverUtils;
} // namespace AT86RF215
#include "at86rf215.hpp"

#include <map>
#include <etl/flat_map.h>

#include "Task.hpp"

namespace AT86RF215 {
static constexpr MorseCodeMapping getMorse(char c) {
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
        case '0': return { 0b11111000, 5 };
        case '1': return { 0b01111000, 5 };
        case '2': return { 0b00111000, 5 };
        case '3': return { 0b00011000, 5 };
        case '4': return { 0b00001000, 5 };
        case '5': return { 0b00000000, 5 };
        case '6': return { 0b10000000, 5 };
        case '7': return { 0b11000000, 5 };
        case '8': return { 0b11100000, 5 };
        case '9': return { 0b11110000, 5 };

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
        case '$': return { 0b00010010, 8 };   // ...-..-
        case '@': return { 0b01101000, 6 };   // .--.-.

        default:
            return { 0, 0 };  // not found
    }
}


    /** =========== Driver's public interface  =========== **/

    State At86rf215_Utilities::get_state(Transceiver transceiver, Error& err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return State::RF_INVALID;
        }
        State state = get_state_private(transceiver, err);
        xSemaphoreGive(resourcesMutexHandle);
        return state;
    }

    void At86rf215_Utilities::set_state(Transceiver transceiver, State state_cmd,
                              Error& err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        if (transceiver == RF09 && transceiverOccupied09) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        if (transceiver == RF24 && transceiverOccupied24) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        set_state_private(transceiver, state_cmd, err);
        xSemaphoreGive(resourcesMutexHandle);
    }

    void At86rf215_Utilities::chip_reset(Error& error) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            error = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        // Chip reset
        spi_write_8(RegisterAddress::RF_RST, 0x07, error);

        // Reset IRQ status registers
        spi_read_8(RegisterAddress::RF09_IRQS, error);
        spi_read_8(RegisterAddress::RF24_IRQS, error);
        spi_read_8(RegisterAddress::BBC0_IRQS, error);
        spi_read_8(RegisterAddress::BBC1_IRQS, error);

        // Restores the current config settings
        setup(error);

        xSemaphoreGive(resourcesMutexHandle);
    }

    etl::expected<void, Error> At86rf215_Utilities::check_transceiver_connection(Error& err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return etl::unexpected(err);
        }

        const DevicePartNumber dpn = get_part_number(err);
        if (err == Error::NO_ERRORS && dpn == DevicePartNumber::AT86RF215) {
            xSemaphoreGive(resourcesMutexHandle);
            return {}; /// success
        } else {
            xSemaphoreGive(resourcesMutexHandle);
            return etl::unexpected<Error>(err);
        }
    }

    int8_t At86rf215_Utilities::clear_channel_assessment(Transceiver transceiver, Error& err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return 0;
        }

        bool& transceiverOccupied = transceiver == RF09 ? transceiverOccupied09 : transceiverOccupied24;

        if (transceiverOccupied) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return 0;
        }

        if (transceiver == RF09) {
            userRequest09 = UserRequest::SINGLE_SHOT_ENERGY_MEASUREMENT;
        } else {
            userRequest24 = UserRequest::SINGLE_SHOT_ENERGY_MEASUREMENT;
        }

        set_state_private(transceiver, State::RF_TXPREP, err);
        xSemaphoreGive(resourcesMutexHandle);

        // wait for the one shot measurement to finish
        if (transceiver == RF09) {
            if (xEventGroupWaitBits(eventGroupHandle,
                energyDetCompletion09GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::SINGLE_SHOT_ENERGY_MEASUREMENT_FAILED;
                return 0;
            }
        } else {
            if (xEventGroupWaitBits(eventGroupHandle,
                energyDetCompletion24GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::SINGLE_SHOT_ENERGY_MEASUREMENT_FAILED;
                return 0;
            }
        }

        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return 0;
        }
        const int8_t energyMeasurement = transceiver == RF09 ? energy_measurement09 : energy_measurement24;
        xSemaphoreGive(resourcesMutexHandle);
        return energyMeasurement;
    }

    // TODO: Upon reaching RX state
    // wait 8μs + RXDFE.SR + Tu
    // read rssi
    void At86rf215_Utilities::packetTransmissionBaseband(Transceiver transceiver,
                                               uint8_t* packet, uint16_t length, Error& err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        bool& transceiverOccupied = transceiver == RF09 ? transceiverOccupied09 : transceiverOccupied24;

        if (transceiverOccupied) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        RegisterAddress regtxflh;
        RegisterAddress regtxfll;
        RegisterAddress regfbtxs;

        if (transceiver == RF09) {
            regtxflh = BBC0_TXFLH;
            regtxfll = BBC0_TXFLL;
            regfbtxs = BBC0_FBTXS;
        } else { // transceiver == RF24
            regtxflh = BBC1_TXFLH;
            regtxfll = BBC1_TXFLL;
            regfbtxs = BBC1_FBTXS;
        }

        // write length to register
        spi_write_8(regtxfll, length & 0xFF, err);
        if (err != Error::NO_ERRORS) {
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }
        spi_write_8(regtxflh, (length >> 8) & 0x07, err);
        if (err != Error::NO_ERRORS) {
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        // write to tx frame buffer
        spi_block_write_8(regfbtxs, length, packet, err);
        if (err != Error::NO_ERRORS) {
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        if (transceiver == RF09) {
            userRequest09 = UserRequest::BASEBAND_TX;
        } else {
            userRequest24 = UserRequest::BASEBAND_TX;
        }

        set_state_private(transceiver, State::RF_TXPREP, err);
        xSemaphoreGive(resourcesMutexHandle);

        // wait for the semaphore tx complete semaphore, to ensure the operation
        // was completed
        if (transceiver == RF09) {
            if (xEventGroupWaitBits(eventGroupHandle,
                basebandTx09GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::TRANSMISSION_FAILED;
            }
        } else {
            if (xEventGroupWaitBits(eventGroupHandle,
                basebandTx24GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::TRANSMISSION_FAILED;
            }
        }
    }

    void At86rf215_Utilities::preparePacketReceptionBaseband(Transceiver transceiver, uint8_t* destBuff, Error &err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        bool& transceiverOccupied = transceiver == RF09 ? transceiverOccupied09 : transceiverOccupied24;

        if (transceiverOccupied) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        if (transceiver == RF09) {
            userRequest09 = UserRequest::BASEBAND_RX;
            destBuffer09 = destBuff;
        } else {
            userRequest24 = UserRequest::BASEBAND_RX;
            destBuffer24 = destBuff;
        }
        set_state_private(transceiver, State::RF_TXPREP, err);
        xSemaphoreGive(resourcesMutexHandle);
    }

    uint16_t At86rf215_Utilities::waitForPacketReceptionBaseband(Transceiver transceiver, Error &err) {
        // wait until a new packet is received
        if (transceiver == RF09) {
            xEventGroupWaitBits(eventGroupHandle,
                   basebandRx09GroupBit,
                     pdTRUE, pdTRUE,
                     portMAX_DELAY);
        } else {
            xEventGroupWaitBits(eventGroupHandle,
                   basebandRx24GroupBit,
                     pdTRUE, pdTRUE,
                     portMAX_DELAY);
        }

        // return the length
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return 0;
        }

        const uint16_t length = transceiver == RF09 ? received_packet_length09 : received_packet_length24;
        xSemaphoreGive(resourcesMutexHandle);
        return length;
    }

    void At86rf215_Utilities::packetTransmissionIQEmbeddedControl(Transceiver transceiver, Error &err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        bool& transceiverOccupied = transceiver == RF09 ? transceiverOccupied09 : transceiverOccupied24;

        if (transceiverOccupied) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        if (transceiver == RF09) {
            userRequest09 = UserRequest::IQ_EEC_TX;
        } else {
            userRequest24 = UserRequest::IQ_EEC_TX;
        }
        set_state_private(transceiver, State::RF_TXPREP, err);
        xSemaphoreGive(resourcesMutexHandle);

        // wait for the baseband processor to end transmission
        if (transceiver == RF09) {
            if (xEventGroupWaitBits(eventGroupHandle,
                iqEecTransmissionComplete09GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::TRANSMISSION_FAILED;
                transceiverOccupied09 = false;
                return;
            }
        } else {
            if (xEventGroupWaitBits(eventGroupHandle,
                iqEecTransmissionComplete24GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::TRANSMISSION_FAILED;
                transceiverOccupied24 = false;
                return;
            }
        }

        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        // Assuming the resource mutex was obtained immediately, we need to wait t_tx_proc_delay for the
        // transceiver to finish transmitting the I/Q samples, and t_pa_ram for the amp to ramp down (figure 7-6).
        // The longest possible wait is  13.75us + 32us = 45.75us (tables 6-1, 6-3).
        uint8_t tries = 0;
        State state;

        // poll the state of the transceiver, until it reaches state TX_PREP
        // TODO maybe use __NOP() in a for loop to wait instead
        do {
            state = get_state_private(transceiver, err);
            tries++;
            if (err != Error::NO_ERRORS) {
                break;
            }
        } while (state == State::RF_TX && tries < 60);

        if (tries >= 60 || err != Error::NO_ERRORS) {
            err = Error::FAILED_CHANGING_STATE;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        // transmission is finished, now the transceiver can be freed up
        if (transceiver == RF09) {
            transceiverOccupied09 = false;
        } else {
            transceiverOccupied24 = false;
        }
        xSemaphoreGive(resourcesMutexHandle);
    }

    void At86rf215_Utilities::preparePacketReceptionIQ(Transceiver transceiver, Error &err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        bool& transceiverOccupied = transceiver == RF09 ? transceiverOccupied09 : transceiverOccupied24;

        if (transceiverOccupied) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        if (transceiver == RF09) {
            userRequest09 = UserRequest::IQ_RX;
        } else {
            userRequest24 = UserRequest::IQ_RX;
        }
        set_state_private(transceiver, State::RF_TXPREP, err);
        xSemaphoreGive(resourcesMutexHandle);
    }

    void At86rf215_Utilities::waitForPacketReceptionIQ(Transceiver transceiver, Error& err) {
        // wait until a preamble is detected
        if (transceiver == RF09) {
            xEventGroupWaitBits(eventGroupHandle,
                                iqPreambleReception09GroupBit,
                                pdTRUE, pdTRUE,
                                portMAX_DELAY);
        } else {
            xEventGroupWaitBits(eventGroupHandle,
                                iqPreambleReception24GroupBit,
                                pdTRUE, pdTRUE,
                                portMAX_DELAY);
        }

        // "lock" the transceiver and freeze the agc
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        if (transceiver == RF09) {
            transceiverOccupied09 = true;
        } else {
            transceiverOccupied24 = true;
        }

        RegisterAddress agcc = transceiver == RF09 ? RegisterAddress::RF09_AGCC : RegisterAddress::RF24_AGCC;

        uint8_t regVal = spi_read_8(agcc, err);
        if (err != Error::NO_ERRORS) {
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        spi_write_8(agcc, regVal | 0x02, err);
        xSemaphoreGive(resourcesMutexHandle);

        // wait for packet reception
        if (transceiver == RF09) {
            if (xEventGroupWaitBits(eventGroupHandle,
                iqPacketReception09GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::RECEPTION_FAILED;
                transceiverOccupied09 = false;
                return;
            }
        } else {
            if (xEventGroupWaitBits(eventGroupHandle,
                iqPacketReception24GroupBit,
                pdTRUE, pdTRUE,
                pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                err = Error::RECEPTION_FAILED;
                transceiverOccupied24 = false;
                return;
            }
        }

        // "unlock" the transceiver and release the AGC
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        regVal = spi_read_8(agcc, err);
        if (err != Error::NO_ERRORS) {
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        spi_write_8(agcc, regVal & 0xFD, err);

        if (transceiver == RF09) {
            transceiverOccupied09 = false;
        } else {
            transceiverOccupied24 = false;
        }

        xSemaphoreGive(resourcesMutexHandle);
    }

    void At86rf215_Utilities::transmitMorseCode(Transceiver transceiver, Error& err, float wpm, const char* sequence, uint16_t sequenceLen) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        bool& transceiverOccupied = transceiver == RF09 ? transceiverOccupied09 : transceiverOccupied24;

        if (transceiverOccupied) {
            err = Error::ONGOING_TRANSMISSION_RECEPTION;
            xSemaphoreGive(resourcesMutexHandle);
            return;
        }

        // setup transceiver as shown in table 13-2 (middle column)
        RegisterAddress iqfc1_reg = RF_IQIFC1;
        RegisterAddress pc_reg;
        RegisterAddress txfhl_reg;
        RegisterAddress txfll_reg;
        RegisterAddress txdaci_reg;
        RegisterAddress txdacq_reg;

        if (transceiver == RF09) {
            pc_reg = BBC0_PC;
            txfhl_reg = BBC0_TXFLH;
            txfll_reg = BBC0_TXFLL;
            txdaci_reg = RF09_TXDACI;
            txdacq_reg = RF09_TXDACQ;
        } else {
            pc_reg = BBC1_PC;
            txfhl_reg = BBC1_TXFLH;
            txfll_reg = BBC1_TXFLL;
            txdaci_reg = RF24_TXDACI;
            txdacq_reg = RF24_TXDACQ;
        }

        const uint8_t iqfc1_val = spi_read_8(iqfc1_reg, err);
        const uint8_t pc_val = spi_read_8(pc_reg, err);
        const uint8_t txfhl_val = spi_read_8(txfhl_reg, err);
        const uint8_t txfll_val = spi_read_8(txfll_reg, err);

        set_state_private(transceiver, State::RF_TRXOFF, err);
        spi_write_8(iqfc1_reg, iqfc1_val & 0x87, err); // CHPM = 0
        spi_write_8(pc_reg, pc_val | 0x80, err);       // CTX = 1
        spi_write_8(txfhl_reg, 0x07, err);             // any length will do
        spi_write_8(txfll_reg, 0xFF, err);
        spi_write_8(txdaci_reg, 0x80 | 0x7E, err); // enable in-phase DAC overwrite with max amplitude
        spi_write_8(txdacq_reg, 0x80 | 0x3F, err); // enable quadrature-phase DAC overwrite with min amplitude

        if (transceiver == RF09) {
            userRequest09 = UserRequest::MORCE_CODE_OOK;
        } else {
            userRequest24 = UserRequest::MORCE_CODE_OOK;
        }
        set_state_private(transceiver, State::RF_TXPREP, err);
        xSemaphoreGive(resourcesMutexHandle);

        const auto timeUnit = static_cast<uint16_t>(1200 / wpm);
        for (uint16_t i = 0; i < sequenceLen; i++) {
            if (sequence[i] == ' ') { // large delay for word gaps
             vTaskDelay(7*pdMS_TO_TICKS(timeUnit));
             continue;
            }

            MorseCodeMapping morseCodeMapping = getMorse(sequence[i]);
            if (morseCodeMapping.dotDashNum == 0) { // skip unknown characters
                continue;
            }

            // transmit character
            for (uint8_t j = 0; j < morseCodeMapping.dotDashNum; j++) {
                if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                    err = Error::RESOURCE_MUTEX_TIMEOUT;
                    return;
                }
                set_state_private(transceiver, State::RF_TX, err);
                xSemaphoreGive(resourcesMutexHandle);

                if (morseCodeMapping.dotDashMapping & (0x80 >> j)) { // dot
                    vTaskDelay(pdMS_TO_TICKS(timeUnit));
                } else {                                             // dash
                    vTaskDelay(3*pdMS_TO_TICKS(timeUnit));
                }

                if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
                    err = Error::RESOURCE_MUTEX_TIMEOUT;
                    return;
                }
                set_state_private(transceiver, State::RF_TXPREP, err);
                xSemaphoreGive(resourcesMutexHandle);


                // delay between character elements
                if (j != morseCodeMapping.dotDashNum - 1) {
                    vTaskDelay(pdMS_TO_TICKS(timeUnit));
                }
            }

            // Delay between characters (skip if next is a space)
            if (i != sequenceLen - 1 && sequence[i + 1] != ' ') {
                vTaskDelay(pdMS_TO_TICKS(3 * timeUnit));
            }
        }

        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        // restore original configuration
        spi_write_8(iqfc1_reg, iqfc1_val, err);
        spi_write_8(pc_reg, pc_val, err);
        spi_write_8(txfhl_reg, txfhl_val, err);
        spi_write_8(txfll_reg, txfll_val, err);
        spi_write_8(txdaci_reg, 0x80 | 0x7E, err); // disable in-phase DAC overwrite
        spi_write_8(txdacq_reg, 0x80 | 0x3F, err); // disable quadrature-phase DAC overwrite

        if (transceiver == RF09) {
            transceiverOccupied09 = false;
        } else {
            transceiverOccupied24 = false;
        }
        err = Error::NO_ERRORS;
        xSemaphoreGive(resourcesMutexHandle);
    }

    void At86rf215_Utilities::print_state(Transceiver transceiver, Error& err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        switch (State rf_state = get_state_private(transceiver, err)) {
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
        xSemaphoreGive(resourcesMutexHandle);
    }

    void At86rf215_Utilities::print_error(Error& err) {
        if (err == Error::NO_ERRORS)
            return;
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

            case Error::UKNOWN_PART_NUMBER:
                LOG_ERROR << "UNKNOWN_PART_NUMBER";
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

            case Error::UKNOWN_DEVICE_PART_NUMBER:
                LOG_ERROR << "UNKNOWN_DEVICE_PART_NUMBER";
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

            case Error::RESOURCE_MUTEX_TIMEOUT:
                LOG_ERROR << "MUTEX_TIMEOUT";
                break;

            case Error::TRANSMISSION_FAILED:
                LOG_ERROR << "BASEBAND_TRANSMISSION_FAILED";

            case Error::RECEPTION_FAILED:
                LOG_ERROR << "BASEBAND_RECEPTION_FAILED";

            case Error::SINGLE_SHOT_ENERGY_MEASUREMENT_FAILED:
                LOG_ERROR << "SINGLE_SHOT_MEASUREMENT_FAILED";

            default:
                LOG_ERROR << "UNHANDLED_ERROR";
                break;
        }
    }

    void At86rf215_Utilities::handle_irq(Error &err) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(mutexTimeout)) != pdTRUE) {
            err = Error::RESOURCE_MUTEX_TIMEOUT;
            return;
        }

        // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        /* Sub 1-GHz Transceiver */

        // Radio IRQ
        volatile uint8_t irq = spi_read_8(RegisterAddress::RF09_IRQS, err);
        if ((irq & InterruptMask::IFSynchronization) != 0) {
            // I/Q IF Synchronization Failure handling
            IFSynchronization_flag = true;
        }
        if ((irq & InterruptMask::TransceiverError) != 0) {
            // Transceiver Error handling
            TransceiverError_flag = true;
        }
        if ((irq & InterruptMask::BatteryLow) != 0) {
            // Battery Low handling
            BatteryLow_flag = true;
        }
        if ((irq & InterruptMask::EnergyDetectionCompletion) != 0) {
            EnergyDetectionCompletion_flag = true;
            // Reenable baseband Core after cca procedure
            if (basebandCoreConfig.baseBandEnable09) {
                uint8_t bbcpc = spi_read_8(BBC0_PC, err);
                spi_write_8(BBC0_PC,(bbcpc & 0xFB) | 0x4, err);
            }

            energy_measurement09 = get_receiver_energy_detection(Transceiver::RF09, err);
            transceiverOccupied09 = false;
            xEventGroupSetBits(eventGroupHandle, energyDetCompletion09GroupBit);
        }
        if ((irq & InterruptMask::TransceiverReady) != 0) {
            TransceiverReady_flag = true;

            // set state rx to receive packets or start measuring energy
            if (userRequest09 == UserRequest::SINGLE_SHOT_ENERGY_MEASUREMENT ||
                userRequest09 == UserRequest::BASEBAND_RX ||
                userRequest09 == UserRequest::IQ_RX) {
                set_state_private(Transceiver::RF09, State::RF_RX, err);
            }

            // Disable baseband core if there is a cca procedure and initialize the
            // single shot measurement.
            if (userRequest09 == UserRequest::SINGLE_SHOT_ENERGY_MEASUREMENT) {
                transceiverOccupied09 = true;
                uint8_t bbcpc = spi_read_8(BBC0_PC,err);
                spi_write_8(BBC0_PC,bbcpc & 0xFB,err);
                spi_write_8(RF09_EDC, static_cast<uint8_t>(EnergyDetectionMode::RF_EDSINGLE), err);
            }

            if (userRequest09 == UserRequest::BASEBAND_TX) {
                transceiverOccupied09 = true;
                set_state_private(Transceiver::RF09, State::RF_TX, err);
            }

            // IQ_EEC_TX: No need to set the state to TX. This is performed automatically when I_DATA[0] == 1
            // MORSE_CODE_OOK: No action needs to be taken, just mark the transceiver as occupied
            if (userRequest09 == UserRequest::IQ_EEC_TX ||
                userRequest09 == UserRequest::MORCE_CODE_OOK) {
                transceiverOccupied09 = true;
            }
        }
        if ((irq & InterruptMask::Wakeup) != 0) {
            Wakeup_flag = true;
            // Wakeup handling
        }

        /// Baseband IRQ
        irq = spi_read_8(RegisterAddress::BBC0_IRQS, err);
        if ((irq & InterruptMask::FrameBufferLevelIndication) != 0) {
            // Frame Buffer Level Indication handling
            FrameBufferLevelIndication_flag = true;
        }
        if ((irq & InterruptMask::AGCRelease) != 0) {
            // AGC Release handling
            AGCRelease_flag = true;
        }
        if ((irq & InterruptMask::AGCHold) != 0) {
            // AGC Hold handling
            AGCHold_flag = true;
            // xTaskNotifyFromISR(rf_rxtask->taskHandle, AGC_HOLD, eSetBits, &xHigherPriorityTaskWoken);
        }
        if ((irq & InterruptMask::TransmitterFrameEnd) != 0) {
            TransmitterFrameEnd_flag = true;
            transceiverOccupied09 = false;

            // notify packetTransmissionBaseband() about successful transmission
            xEventGroupSetBits(eventGroupHandle, basebandTx09GroupBit);
        }
        if ((irq & InterruptMask::ReceiverExtendMatch) != 0) {
            // Receiver Extended Match handling
            ReceiverExtendMatch_flag = true;
        }
        if ((irq & InterruptMask::ReceiverAddressMatch) != 0) {
            // Receiver Address Match handling
            ReceiverAddressMatch_flag = true;
        }
        if ((irq & InterruptMask::ReceiverFrameEnd) != 0) {
            ReceiverFrameEnd_flag = true;
            RegisterAddress regrxflh = BBC0_RXFLH;
            RegisterAddress regrxfll = BBC0_RXFLL;
            RegisterAddress regfbrxs = BBC0_FBRXS;
            received_packet_length09 = (spi_read_8(regrxflh, err) << 8) | static_cast<uint16_t>(spi_read_8(regrxfll, err));
            spi_block_read_8(regfbrxs, received_packet_length09, destBuffer09, err);
            transceiverOccupied09 = false;

            // notify packetReceptionBaseband()
            xEventGroupSetBits(eventGroupHandle, basebandRx09GroupBit);
        }
        if ((irq & InterruptMask::ReceiverFrameStart) != 0) {
            ReceiverFrameStart_flag = true;
            transceiverOccupied09 = true;
        }

        /* 2.4 GHz Transceiver */

        // Radio IRQ
        irq = spi_read_8(RegisterAddress::RF24_IRQS, err);

        if ((irq & InterruptMask::IFSynchronization) != 0) {
            // I/Q IF Synchronization Failure handling
            IFSynchronization_flag = true;
        }
        if ((irq & InterruptMask::TransceiverError) != 0) {
            // Transceiver Error handling
            TransceiverError_flag = true;
        }
        if ((irq & InterruptMask::BatteryLow) != 0) {
            BatteryLow_flag = true;
            // Battery Low handling
        }
        if ((irq & InterruptMask::EnergyDetectionCompletion) != 0) {
            EnergyDetectionCompletion_flag = true;
            // Reenable baseband Core after cca procedure
            if (basebandCoreConfig.baseBandEnable24) {
                uint8_t bbcpc = spi_read_8(BBC1_PC, err);
                spi_write_8(BBC1_PC,(bbcpc & 0xFB) | 0x4, err);
            }

            energy_measurement24 = get_receiver_energy_detection(Transceiver::RF24, err);
            transceiverOccupied24 = false;
            xEventGroupSetBits(eventGroupHandle, energyDetCompletion24GroupBit);
        }
        if ((irq & InterruptMask::TransceiverReady) != 0) {
            TransceiverReady_flag = true;

            // set state rx to receive packets or start measuring energy
            if (userRequest24 == UserRequest::SINGLE_SHOT_ENERGY_MEASUREMENT ||
                userRequest24 == UserRequest::BASEBAND_RX ||
                userRequest24 == UserRequest::IQ_RX) {
                set_state_private(Transceiver::RF24, State::RF_RX, err);
                }

            // Disable baseband core if there is a cca procedure and initialize the
            // single shot measurement.
            if (userRequest24 == UserRequest::SINGLE_SHOT_ENERGY_MEASUREMENT) {
                transceiverOccupied24 = true;
                uint8_t bbcpc = spi_read_8(BBC1_PC,err);
                spi_write_8(BBC1_PC,bbcpc & 0xFB,err);
                spi_write_8(RF24_EDC, static_cast<uint8_t>(EnergyDetectionMode::RF_EDSINGLE), err);
            }

            if (userRequest24 == UserRequest::BASEBAND_TX) {
                transceiverOccupied24 = true;
                set_state_private(Transceiver::RF24, State::RF_TX, err);
            }

            // IQ_EEC_TX: No need to set the state to TX. This is performed automatically when I_DATA[0] == 1
            // MORSE_CODE_OOK: No action needs to be taken, just mark the transceiver as occupied
            if (userRequest24 == UserRequest::IQ_EEC_TX ||
                userRequest24 == UserRequest::MORCE_CODE_OOK) {
                transceiverOccupied09 = true;
            }
        }
        if ((irq & InterruptMask::Wakeup) != 0) {
            // Wakeup handling
            Wakeup_flag = true;
        }

        //Baseband IRQ
        irq = spi_read_8(RegisterAddress::BBC1_IRQS, err);
        if ((irq & InterruptMask::FrameBufferLevelIndication) != 0) {
            // Frame Buffer Level Indication handling
            FrameBufferLevelIndication_flag = true;
        }
        if ((irq & InterruptMask::AGCRelease) != 0) {
            // AGC Release handling
            AGCRelease_flag = true;
        }
        if ((irq & InterruptMask::AGCHold) != 0) {
            AGCHold_flag = true;
        }
        if ((irq & InterruptMask::TransmitterFrameEnd) != 0) {
            TransmitterFrameEnd_flag = true;
            transceiverOccupied24 = false;

            // notify packetTransmissionBaseband() about successful transmission
            xEventGroupSetBits(eventGroupHandle, basebandTx24GroupBit);
        }
        if ((irq & InterruptMask::ReceiverExtendMatch) != 0) {
            // Receiver Extended Match handling
            ReceiverExtendMatch_flag = true;
        }
        if ((irq & InterruptMask::ReceiverAddressMatch) != 0) {
            // Receiver Address Match handling
            ReceiverAddressMatch_flag = true;
        }
        if ((irq & InterruptMask::ReceiverFrameEnd) != 0) {
            ReceiverFrameEnd_flag = true;
            RegisterAddress regrxflh = BBC1_RXFLH;
            RegisterAddress regrxfll = BBC1_RXFLL;
            RegisterAddress regfbrxs = BBC1_FBRXS;
            received_packet_length24 = (spi_read_8(regrxflh, err) << 8) | static_cast<uint16_t>(spi_read_8(regrxfll, err));
            spi_block_read_8(regfbrxs, received_packet_length24, destBuffer24, err);
            transceiverOccupied24 = false;

            // notify packetReceptionBaseband()
            xEventGroupSetBits(eventGroupHandle, basebandRx24GroupBit);
        }
        if ((irq & InterruptMask::ReceiverFrameStart) != 0) {
            ReceiverFrameStart_flag = true;
            transceiverOccupied24 = true;
        }

        xSemaphoreGive(resourcesMutexHandle);
    }

    /** =========== Private functions  =========== **/

    void At86rf215_Utilities::spi_write_8(uint16_t address, uint8_t value, Error& err) {
        uint8_t msg[3] = {static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)), static_cast<uint8_t>(address & 0xFF), value};

        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET); // slave select pin
        uint8_t hal_error = HAL_SPI_Transmit_DMA(hspi, msg, 3);

        if (hal_error != HAL_OK ||
            xEventGroupWaitBits(eventGroupHandle,
                                spiWriteCompleteGroupBit,
                                pdTRUE, pdTRUE,
                                mutexTimeout) != pdTRUE) {
            HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
            err = Error::FAILED_WRITING_TO_REGISTER;
            return;
        }

        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;
    }

    uint8_t At86rf215_Utilities::spi_read_8(uint16_t address, Error& err) {
        uint8_t msg[2] = {static_cast<uint8_t>((address >> 8) & 0x7F), static_cast<uint8_t>(address & 0xFF)};
        uint8_t response[3];
        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET); // slave select pin
        uint8_t hal_error = HAL_SPI_TransmitReceive_DMA(hspi, msg, response, 3);

        if (hal_error != HAL_OK ||
            xEventGroupWaitBits(eventGroupHandle,
                            spiReadCompleteGroupBit,
                            pdTRUE, pdTRUE,
                            mutexTimeout) != pdTRUE) {
            HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
            err = Error::FAILED_READING_FROM_REGISTER;
            return 0;
        }

        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;

        return response[2];
    }

    void At86rf215_Utilities::spi_block_write_8(uint16_t address, uint16_t n, uint8_t* value,
                                      Error& err) {
        uint8_t msg[2] = {static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)), static_cast<uint8_t>(address & 0xFF)};
        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET); // slave select pin

        uint8_t hal_error = HAL_SPI_Transmit_DMA(hspi, msg, 2);
        if (hal_error != HAL_OK ||
            xEventGroupWaitBits(eventGroupHandle,
                            spiWriteCompleteGroupBit,
                            pdTRUE, pdTRUE,
                            mutexTimeout) != pdTRUE) {
            HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
            err = Error::FAILED_WRITING_TO_REGISTER;
            return;
        }

        hal_error = HAL_SPI_Transmit_DMA(hspi, value, n);
        if (hal_error != HAL_OK ||
            xEventGroupWaitBits(eventGroupHandle,
                            spiWriteCompleteGroupBit,
                            pdTRUE, pdTRUE,
                            mutexTimeout) != pdTRUE) {
            HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
            err = Error::FAILED_WRITING_TO_REGISTER;
            return;
        }

        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

        err = Error::NO_ERRORS;
    }

    uint8_t* At86rf215_Utilities::spi_block_read_8(uint16_t address, uint8_t n,
                                         uint8_t* response, Error& err) {
        uint8_t msg[2] = {static_cast<uint8_t>((address >> 8) & 0x7F), static_cast<uint8_t>(address & 0xFF)};

        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET); // slave select pin
        uint8_t hal_error = HAL_SPI_TransmitReceive_DMA(hspi, msg, response, n + 2);
        if (hal_error != HAL_OK ||
        xEventGroupWaitBits(eventGroupHandle,
                            spiReadCompleteGroupBit,
                            pdTRUE, pdTRUE,
                            mutexTimeout) != pdTRUE) {
            HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
            err = Error::FAILED_READING_FROM_REGISTER;
            return response;
        }

        HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;
        return response + 2;
    }

    State At86rf215_Utilities::get_state_private(Transceiver transceiver, Error& err) {
        uint8_t state;
        if (transceiver == RF09) {
            state = spi_read_8(RF09_STATE, err) & 0x07;
        } else { // transceiver == RF24
            state = spi_read_8(RF24_STATE, err) & 0x07;
        }

        if (err != Error::NO_ERRORS) {
            return State::RF_INVALID;
        }
        err = Error::NO_ERRORS;

        if ((state >= 0x02) && (state <= 0x07)) {
            return static_cast<State>(state);
        } else {
            err = Error::UKNOWN_REQUESTED_STATE;
            return State::RF_INVALID;
        }
    }

    void At86rf215_Utilities::set_state_private(Transceiver transceiver, State state_cmd,
                              Error& err) {
        State state = get_state_private(transceiver, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        err = Error::NO_ERRORS;

        switch (state_cmd) {
            case State::RF_TRXOFF:
                break;
            case State::RF_TXPREP:
                if ((state != State::RF_TRXOFF) && (state != State::RF_RX) && (state != State::RF_TX)) {
                    err = Error::FAILED_CHANGING_STATE;
                    return;
                }
                break;
            case State::RF_TX:
            case State::RF_RX:
                if (state != State::RF_TXPREP) {
                    err = Error::FAILED_CHANGING_STATE;
                    return;
                }
                break;
            case State::RF_NOP:
                break;
            case State::RF_RESET:
                break;
            case State::RF_SLEEP:
                if ((state != State::RF_TRXOFF) && (state != State::RF_SLEEP)) {
                    err = Error::FAILED_CHANGING_STATE;
                    return;
                }
                break;
            default:
                err = Error::FAILED_CHANGING_STATE;
                return;
        }

        if (transceiver == RF09) {
            spi_write_8(RF09_CMD, static_cast<uint8_t>(state_cmd), err);
        } else { // transceiver == RF24
            spi_write_8(RF24_CMD, static_cast<uint8_t>(state_cmd), err);
        }
    }

    void At86rf215_Utilities::set_pll_channel_spacing(Transceiver transceiver,
                                            uint8_t spacing, Error& err) {
        RegisterAddress regscs;

        if (transceiver == RF09) {
            regscs = RF09_CS;
        } else { // transceiver == RF24
            regscs = RF24_CS;
        }
        spi_write_8(regscs, spacing, err);
    }

    uint8_t At86rf215_Utilities::get_pll_channel_spacing(Transceiver transceiver,
                                               Error& err) {
        RegisterAddress regscs;

        if (transceiver == RF09) {
            regscs = RF09_CS;
        } else { // transceiver == RF24
            regscs = RF24_CS;
        }
        return spi_read_8(regscs, err);
    }

    void At86rf215_Utilities::set_pll_channel_frequency(Transceiver transceiver,
                                              uint16_t freq, Error& err) {
        RegisterAddress regcf0h;
        RegisterAddress regcf0l;

        if (transceiver == RF09) {
            regcf0h = RF09_CCF0H;
            regcf0l = RF09_CCF0L;
        } else { // transceiver == RF24
            regcf0h = RF24_CCF0H;
            regcf0l = RF24_CCF0L;
        }

        spi_write_8(regcf0l, freq & 0x00FF, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        spi_write_8(regcf0h, (freq & 0xFF00) >> 8, err);
    }

    uint16_t At86rf215_Utilities::get_pll_channel_frequency(Transceiver transceiver,
                                                  Error& err) {
        RegisterAddress regcf0h;
        RegisterAddress regcf0l;

        if (transceiver == RF09) {
            regcf0h = RF09_CCF0H;
            regcf0l = RF09_CCF0L;
        } else { // transceiver == RF24
            regcf0h = RF24_CCF0H;
            regcf0l = RF24_CCF0L;
        }

        uint16_t cf0h = spi_read_8(regcf0h, err) & 0xFF;
        if (err != Error::NO_ERRORS) {
            return 0;
        }
        uint16_t cf0l = spi_read_8(regcf0l, err) & 0xFF;

        return (cf0h << 8) | cf0l;
    }

    uint16_t At86rf215_Utilities::get_pll_channel_number(Transceiver transceiver,
                                               Error& err) {
        RegisterAddress regcnh;
        RegisterAddress regcnl;

        if (transceiver == RF09) {
            regcnh = RF09_CNM;
            regcnl = RF09_CNL;
        } else { // transceiver == RF24
            regcnh = RF24_CNM;
            regcnl = RF24_CNL;
        }

        uint16_t cnh = spi_read_8(regcnh, err) & 0x0100;
        if (err != Error::NO_ERRORS) {
            return 0;
        }
        uint16_t cnl = spi_read_8(regcnl, err) & 0x00FF;

        return (cnh << 8) | cnl;
    }

    void At86rf215_Utilities::set_pll_bw(PLLBandwidth bw, Error& err) {
        uint8_t reg_pll = spi_read_8(RF09_PLL, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // Clear bits [5:4] and set new value
        reg_pll &= ~(0x3 << 4);                     // Clear bits [5:4] (0x3 << 4 = 0b0011 0000)
        reg_pll |= (static_cast<uint8_t>(bw) << 4); // Set new value for bits [5:4]

        spi_write_8(RF09_PLL, reg_pll, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
    }

    PLLBandwidth At86rf215_Utilities::get_pll_bw(Error& err) {
        uint8_t bw = (spi_read_8(RF09_PLL, err) >> 4) & 0x03;
        if (err != Error::NO_ERRORS) {
            return PLLBandwidth::BWInvalid;
        }
        return static_cast<PLLBandwidth>(bw);
    }

    PLLState At86rf215_Utilities::get_pll_state(Transceiver transceiver, Error& err) {
        RegisterAddress regpll;

        if (transceiver == RF09) {
            regpll = RF09_PLL;
        } else { // transceiver == RF24
            regpll = RF24_PLL;
        }

        uint8_t state = (spi_read_8(regpll, err) >> 1) & 0x01;
        return static_cast<PLLState>(state);
    }

    void At86rf215_Utilities::configure_pll(Transceiver transceiver, FrequencySynthesizerConfig& frequencySynthesizerConfig, Error& err) {

        if (get_state_private(transceiver, err) != State::RF_TRXOFF) {
            err = Error::INVALID_STATE_FOR_OPERATION;
            return;
        }

        bool validConfigFlag;
        PLLChannelMode channelMode;
        uint32_t freq;
        PLLBandwidth bw;
        RegisterAddress ccf0h;
        RegisterAddress ccf0l;
        RegisterAddress cnm;
        RegisterAddress cs;
        RegisterAddress cnl;
        if (transceiver == RF09) {
            validConfigFlag = frequencySynthesizerConfig.validConfig09;
            channelMode = freqSynthesizerConfig.channelMode09;
            freq = frequencySynthesizerConfig.frequency09;
            bw = frequencySynthesizerConfig.loopBandwidth09;
            ccf0h = RF09_CCF0H;
            ccf0l = RF09_CCF0L;
            cnm = RF09_CNM;
            cs = RF09_CS;
            cnl = RF09_CNL;
        } else {
            validConfigFlag = frequencySynthesizerConfig.validConfig24;
            channelMode = freqSynthesizerConfig.channelMode24;
            freq = frequencySynthesizerConfig.frequency24;
            bw = frequencySynthesizerConfig.loopBandwidth24;
            ccf0h = RF24_CCF0H;
            ccf0l = RF24_CCF0L;
            cnm = RF24_CNM;
            cs = RF24_CS;
            cnl = RF24_CNL;
        }

        if (!validConfigFlag) {
            err =  Error::INVALID_TRANSCEIVER_FREQ;
            return;
        }

        // configure channel mode
        uint8_t cnmVal = spi_read_8(cnm, err);
        spi_write_8(cnm, cnmVal | (static_cast<uint8_t >(channelMode) << 6), err);
        if (channelMode == PLLChannelMode::IEECompliant) {
            // RFn_CCF0H, RFn_CCFOL:  high and low byte of central frequency
            // CNM.CHN, RFn_CNL: high bit and low byte of channel number
            // RFn_CS: channel spacing
            // @TODO: central frequency and channel spacing for each band in 68d, 68e tables of IEEE Std 802.15.4g™-2012
            err = Error::INVALID_TRANSCEIVER_FREQ;
            return;
        } else {
            // RFn_CCF0H, RFn_CCF0L, RFn_CNL: high, middle and low byte of N_channnel
            uint32_t Nchannel;
            if (channelMode == PLLChannelMode::FineResolution450) {
                Nchannel = (freq - 377000) * 65536 / 6500;
            }
            else if (channelMode == PLLChannelMode::FineResolution450) {
                Nchannel = (freq - 754000) * 65536 / 13000;
            }
            else {
                Nchannel = (freq - 2366000) * 65536 / 26000;
            }
            spi_write_8(ccf0h, static_cast<uint8_t>(Nchannel >> 16), err);
            spi_write_8(ccf0l, static_cast<uint8_t>(Nchannel >> 8), err);
            spi_write_8(cnl, static_cast<uint8_t>(Nchannel), err);
        }

        /// RFn_PLL
        set_pll_bw(bw, err);
    }

    DevicePartNumber At86rf215_Utilities::get_part_number(Error& err) {
        uint8_t dpn = spi_read_8(RegisterAddress::RF_PN, err);
        if (err != Error::NO_ERRORS) {
            return DevicePartNumber::AT86RF215_INVALID;
        }

        if ((dpn >= 0x34) && (dpn <= 0x36)) {
            return static_cast<DevicePartNumber>(dpn);
        } else {
            return DevicePartNumber::AT86RF215_INVALID;
        }
    }

    DeviceVersionNumber At86rf215_Utilities::get_version_number(Error& err) {
        uint8_t vn = spi_read_8(RegisterAddress::RF_VN, err);
        if (err != Error::NO_ERRORS) {
            return DeviceVersionNumber::INVALID_VERSION_NUMBER;
        }
        return static_cast<DeviceVersionNumber>(vn);
    }

    uint8_t At86rf215_Utilities::get_pll_frequency(Transceiver transceiver, Error& err) {
        RegisterAddress regpll;

        if (transceiver == RF09) {
            regpll = RF09_PLLCF;
        } else { // transceiver == RF24
            regpll = RF24_PLLCF;
        }

        uint8_t freq = spi_read_8(regpll, err) & 0x3F;
        if (err != Error::NO_ERRORS) {
            return 0;
        }
        return freq;
    }

    void At86rf215_Utilities::set_tcxo_trimming(CrystalTrim trim, Error& err) {
        uint8_t trgxcov = spi_read_8(RF_XOC, err) & 0x1F;
        if (err != Error::NO_ERRORS)
            return;
        spi_write_8(RF_XOC, (trgxcov & 0x10) | (static_cast<uint8_t>(trim) & 0x0F),
                    err);
    }

    CrystalTrim At86rf215_Utilities::read_tcxo_trimming(Error& err) {
        auto crystalTrim =
                static_cast<CrystalTrim>(spi_read_8(RF_XOC, err) & 0x0F);
        if (err != Error::NO_ERRORS) {
            return CrystalTrim::TRIM_INV;
        }
        return crystalTrim;
    }

    void At86rf215_Utilities::set_tcxo_fast_start_up_enable(bool fast_start_up, Error& err) {
        uint8_t trgxcov = spi_read_8(RF_XOC, err) & 0x1F;
        if (err != Error::NO_ERRORS)
            return;
        spi_write_8(RF_XOC, (trgxcov & 0x0F) | (fast_start_up << 4), err);
    }

    bool At86rf215_Utilities::read_tcxo_fast_start_up_enable(Error& err) {
        bool fast_start_up =
                static_cast<bool>((spi_read_8(RF_XOC, err) & 0x10) >> 4);
        return fast_start_up;
    }

    PowerAmplifierRampTime At86rf215_Utilities::get_pa_ramp_up_time(Transceiver transceiver,
                                                          Error& err) {
        RegisterAddress regtxcutc;

        if (transceiver == RF09) {
            regtxcutc = RF09_TXCUTC;
        } else { // transceiver == RF24
            regtxcutc = RF24_TXCUTC;
        }

        uint8_t ramp = spi_read_8(regtxcutc, err) & 0xC0 >> 6;
        return static_cast<PowerAmplifierRampTime>(ramp);
    }

    TransmitterCutOffFrequency At86rf215_Utilities::get_cutoff_freq(Transceiver transceiver,
                                                          Error& err) {
        RegisterAddress regtxcutc;

        if (transceiver == RF09) {
            regtxcutc = RF09_TXCUTC;
        } else { // transceiver == RF24
            regtxcutc = RF24_TXCUTC;
        }

        uint8_t cutoff = spi_read_8(regtxcutc, err) & 0x0F;
        return static_cast<TransmitterCutOffFrequency>(cutoff);
    }


    TxRelativeCutoffFrequency At86rf215_Utilities::get_relative_cutoff_freq(
            Transceiver transceiver, Error& err) {
        RegisterAddress regtxdfe;

        if (transceiver == RF09) {
            regtxdfe = RF09_TXDFE;
        } else { // transceiver == RF24
            regtxdfe = RF24_TXDFE;
        }

        uint8_t dfe = (spi_read_8(regtxdfe, err) & 0xE0) >> 5;
        return static_cast<TxRelativeCutoffFrequency>(dfe);
    }


    bool At86rf215_Utilities::get_direct_modulation(Transceiver transceiver, Error& err) {
        RegisterAddress regtxdfe;

        if (transceiver == RF09) {
            regtxdfe = RF09_TXDFE;
        } else { // transceiver == RF24
            regtxdfe = RF24_TXDFE;
        }

        return (spi_read_8(regtxdfe, err) & 0x10) >> 4;
    }


    ReceiverSampleRate At86rf215_Utilities::get_sample_rate(Transceiver transceiver,
                                                  Error& err) {
        RegisterAddress regtxdfe;

        if (transceiver == RF09) {
            regtxdfe = RF09_TXDFE;
        } else { // transceiver == RF24
            regtxdfe = RF24_TXDFE;
        }

        return static_cast<ReceiverSampleRate>(spi_read_8(regtxdfe, err) & 0x0F);
    }

    PowerAmplifierCurrentControl At86rf215_Utilities::get_pa_dc_current(
            Transceiver transceiver, Error& err) {
        RegisterAddress regpac;

        if (transceiver == RF09) {
            regpac = RF09_PAC;
        } else { // transceiver == RF24
            regpac = RF24_PAC;
        }

        uint8_t txpa = spi_read_8(regpac, err) & 0x60 >> 5;
        return static_cast<PowerAmplifierCurrentControl>(txpa);
    }


    bool At86rf215_Utilities::get_lna_bypassed(Transceiver transceiver, Error& err) {
        RegisterAddress regaux =
                (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
        uint8_t lna_bypass = spi_read_8(regaux, err) & 0x80;
        if (err != Error::NO_ERRORS)
            return 0;
        return lna_bypass >> 7;
    }

    AutomaticGainControlMAP At86rf215_Utilities::get_agcmap(Transceiver transceiver,
                                                  Error& err) {
        RegisterAddress regaux =
                (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
        uint8_t agcmap = spi_read_8(regaux, err) & 0x60;
        if (err != Error::NO_ERRORS)
            return AutomaticGainControlMAP::AGC_INVALID;
        return static_cast<AutomaticGainControlMAP>(agcmap >> 5);
    }


    AutomaticVoltageExternal At86rf215_Utilities::get_external_analog_voltage(
            Transceiver transceiver, Error& err) {
        RegisterAddress regaux =
                (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
        uint8_t agcmap = spi_read_8(regaux, err) & 0x10;
        if (err != Error::NO_ERRORS)
            return AutomaticVoltageExternal::INVALID;
        return static_cast<AutomaticVoltageExternal>(agcmap >> 4);
    }

    bool At86rf215_Utilities::get_analog_voltage_settled_status(Transceiver transceiver,
                                                      Error& err) {
        RegisterAddress regaux =
                (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
        uint8_t avs = spi_read_8(regaux, err) & 0x04;
        if (err != Error::NO_ERRORS)
            return 0;
        return avs >> 2;
    }
    PowerAmplifierVoltageControl At86rf215_Utilities::get_analog_power_amplifier_voltage(
            Transceiver transceiver, Error& err) {
        RegisterAddress regaux =
                (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
        uint8_t pavc = spi_read_8(regaux, err) & 0x03;
        if (err != Error::NO_ERRORS)
            return PowerAmplifierVoltageControl::PAVC_INVALID;
        return static_cast<PowerAmplifierVoltageControl>(pavc);
    }


    void At86rf215_Utilities::set_ed_average_detection(Transceiver transceiver, uint8_t df,
                                             EnergyDetectionTimeBasis dtb, Error& err) {
        RegisterAddress regedd;

        if (transceiver == RF09) {
            regedd = RF09_EDD;
        } else { // transceiver == RF24
            regedd = RF24_EDD;
        }

        uint8_t reg = ((df & 0xCF) << 2) | (static_cast<uint8_t>(dtb) & 0x3);
        spi_write_8(regedd, reg, err);
    }

    uint8_t At86rf215_Utilities::get_ed_average_detection(Transceiver transceiver,
                                                Error& err) {
        RegisterAddress regedd;

        if (transceiver == RF09) {
            regedd = RF09_EDD;
        } else { // transceiver == RF24
            regedd = RF24_EDD;
        }

        uint8_t reg = spi_read_8(regedd, err);
        uint8_t df = (reg & 0xFC) >> 2;
        uint8_t dtb = (reg & 0x3);
        return df * dtb;
    }

    int8_t At86rf215_Utilities::get_receiver_energy_detection(Transceiver transceiver,
                                                    Error& err) {
        RegisterAddress regedv;

        if (transceiver == RF09) {
            regedv = RF09_EDV;
        } else { // transceiver == RF24
            regedv = RF24_EDV;
        }

        int8_t reg = static_cast<int8_t>(spi_read_8(regedv, err));

        if (err != Error::NO_ERRORS) {
            return 127;
        }
        if (reg > 4) {
            err = Error::INVALID_RSSI_MEASUREMENT;
            return 127;
        }
        return reg;
    }

    void At86rf215_Utilities::set_battery_monitor_control(BatteryMonitorHighRange range, BatteryMonitorVoltageThreshold threshold, Error& err) {
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_battery_monitor_high_range(range, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_battery_monitor_voltage_threshold(threshold, err);
    }


    BatteryMonitorStatus At86rf215_Utilities::get_battery_monitor_status(Error& err) {
        uint8_t status = (spi_read_8(RF_BMDVC, err) & 0x20) >> 5;
        return static_cast<BatteryMonitorStatus>(status);
    }

    void At86rf215_Utilities::set_battery_monitor_high_range(BatteryMonitorHighRange range,
                                                   Error& err) {
        uint8_t bmhr = spi_read_8(RF_BMDVC, err) & 0x2F;
        if (err != Error::NO_ERRORS)
            return;
        spi_write_8(RF_BMDVC, (static_cast<uint8_t>(range) << 4) | bmhr, err);
    }

    uint8_t At86rf215_Utilities::get_battery_monitor_high_range(Error& err) {
        return (spi_read_8(RF_BMDVC, err) & 0x10) >> 4;
    }

    void At86rf215_Utilities::set_battery_monitor_voltage_threshold(
            BatteryMonitorVoltageThreshold threshold, Error& err) {
        uint8_t reg_value_bmvt = spi_read_8(RF_BMDVC, err);
        reg_value_bmvt &= ~(0xF);
        if (err != Error::NO_ERRORS)
            return;
        spi_write_8(RF_BMDVC, reg_value_bmvt | static_cast<uint8_t>(threshold), err);
    }

    uint8_t At86rf215_Utilities::get_battery_monitor_voltage_threshold(Error& err) {
        return spi_read_8(RF_BMDVC, err) & 0x0F;
    }

    void At86rf215_Utilities::set_external_front_end_control(Transceiver transceiver, ExternalFrontEndControl frontEndControl, Error& err) {
        RegisterAddress reg_address;
        if (transceiver == RF09)
            reg_address = RF09_PADFE;
        else if (transceiver == RF24)
            reg_address = RF24_PADFE;
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS)
            return;
        // clears the bits [7:6]
        reg_value &= ~(0x3 << 6);
        reg_value |= static_cast<uint8_t>(frontEndControl) << 6;
        spi_write_8(reg_address, reg_value, err);
    }

    void At86rf215_Utilities::setup_tx_frontend(Transceiver transceiver,
                                      PowerAmplifierRampTime pa_ramp_time, TransmitterCutOffFrequency cutoff,
                                      TxRelativeCutoffFrequency tx_rel_cutoff, Direct_Mod_Enable_FSKDM direct_mod,
                                      TransmitterSampleRate tx_sample_rate,
                                      PowerAmplifierCurrentControl pa_curr_control, uint8_t tx_out_power,
                                      ExternalLNABypass ext_lna_bypass, AutomaticGainControlMAP agc_map,
                                      AutomaticVoltageExternal avg_ext, AnalogVoltageEnable av_enable,
                                      PowerAmplifierVoltageControl pa_vcontrol, ExternalFrontEndControl externalFrontEndControl, Error& err) {
        RegisterAddress regtxcut;
        RegisterAddress regtxdfe;
        RegisterAddress regpac;
        RegisterAddress regauxs;

        uint8_t reg = 0;

        if (transceiver == Transceiver::RF09) {
            regtxcut = RF09_TXCUTC;
            regtxdfe = RF09_TXDFE;
            regpac = RF09_PAC;
            regauxs = RF09_AUXS;
        } else if (transceiver == Transceiver::RF24) {
            regtxcut = RF24_TXCUTC;
            regtxdfe = RF24_TXDFE;
            regpac = RF24_PAC;
            regauxs = RF24_AUXS;
        }
        // Set RFn_TXCUTC
        reg = (static_cast<uint8_t>(pa_ramp_time) << 6) | static_cast<uint8_t>(cutoff);
        spi_write_8(regtxcut, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        // Set RFn_TXDFE
        reg = (static_cast<uint8_t>(tx_rel_cutoff) << 5) | static_cast<uint8_t>(direct_mod) << 4 | static_cast<uint8_t>(tx_sample_rate);
        spi_write_8(regtxdfe, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // Set RFn_PAC
        reg = (static_cast<uint8_t>(pa_curr_control) << 5) | (tx_out_power & 0x1F);
        spi_write_8(regpac, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        // Set RFn_AUXS
        reg = (static_cast<uint8_t>(ext_lna_bypass) << 7) | (static_cast<uint8_t>(agc_map) << 5) | (static_cast<uint8_t>(avg_ext) << 4) | (static_cast<uint8_t>(av_enable) << 3) | (static_cast<uint8_t>(pa_vcontrol));
        spi_write_8(regauxs, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // Set RFn_PADFE
        set_external_front_end_control(transceiver, externalFrontEndControl, err);
    }

    void At86rf215_Utilities::setup_iq(ExternalLoopback external_loop,
                             IQOutputCurrent out_cur, IQmodeVoltage common_mode_vol,
                             IQmodeVoltageIEE common_mode_iee, EmbeddedControlTX embedded_tx_start,
                             ChipMode chip_mode, SkewAlignment skew_alignment, Error& err) {
        // Set RF_IQIFC0
        uint8_t reg;
        reg = (static_cast<uint8_t>(external_loop) << 7) | (static_cast<uint8_t>(out_cur) << 4) | (static_cast<uint8_t>(common_mode_vol) << 2) | (static_cast<uint8_t>(common_mode_iee) << 1) | static_cast<uint8_t>(embedded_tx_start);
        spi_write_8(RF_IQIFC0, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        // Set RF_IQIFC1
        reg = (static_cast<uint8_t>(chip_mode) << 4) | static_cast<uint8_t>(skew_alignment);
        spi_write_8(RF_IQIFC1, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
    }

    bool At86rf215_Utilities::get_iqSyncStatus(Error& err) {
        RegisterAddress reg = RegisterAddress::RF_IQIFC2;
        uint8_t val = spi_read_8(reg, err);
        if (err != Error::NO_ERRORS) {
            return false;
        }
        return val >> 7;
    }

    void At86rf215_Utilities::setup_crystal(bool fast_start_up, CrystalTrim crystal_trim,
                                  Error& err) {
       set_tcxo_fast_start_up_enable(fast_start_up, err);
       if (err != Error::NO_ERRORS) {
           return;
       }

       set_tcxo_trimming(crystal_trim, err);
       if (err != Error::NO_ERRORS) {
           return;
       }
    }

    void At86rf215_Utilities::setup_rx_energy_detection(Transceiver transceiver,
                                              EnergyDetectionMode energy_mode, uint8_t energy_detect_factor,
                                              EnergyDetectionTimeBasis energy_time_basis, Error& err) {
        uint8_t reg_value;
        RegisterAddress regedc;
        RegisterAddress regedd;

        if (transceiver == Transceiver::RF09) {
            regedc = RF09_EDC;
            regedd = RF09_EDD;
        } else if (transceiver == Transceiver::RF24) {
            regedc = RF24_EDC;
            regedd = RF24_EDD;
        }
        // Read the current register value
        reg_value = spi_read_8(regedc, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        // Set RFn_EDC
        // Clear bits [1:0] and update them with the new value
        reg_value &= ~0x03;                                    // Clear bits [1:0] (0x03 = 0000 0011)
        reg_value |= static_cast<uint8_t>(energy_mode) & 0x03; // Write new value to bits [1:0]
        spi_write_8(regedc, reg_value, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // Set RFn_EDD
        reg_value = (energy_detect_factor << 2) | static_cast<uint8_t>(energy_time_basis);
        spi_write_8(regedd, reg_value, err);
    }

    void At86rf215_Utilities::setup_rx_frontend(Transceiver transceiver, bool if_inversion,
                                      bool if_shift, ReceiverBandwidth rx_bw,
                                      RxRelativeCutoffFrequency rx_rel_cutoff,
                                      ReceiverSampleRate rx_sample_rate, bool agc_input,
                                      AverageTimeNumberSamples agc_avg_sample, AGCReset agc_reset, AGCFreezeControl agc_freeze_control, AGCEnable agc_enable,
                                      AutomaticGainTarget agc_target, uint8_t gain_control_word, Error& err) {
        if (gain_control_word > 23) {
            err = Error::INVALID_AGC_CONTROl_WORD;
            return;
        }

        RegisterAddress regrxbwc;
        RegisterAddress regrxdfe;
        RegisterAddress regagcc;
        RegisterAddress regagcs;

        uint8_t reg = 0;

        if (transceiver == Transceiver::RF09) {
            regrxbwc = RF09_RXBWC;
            regrxdfe = RF09_RXDFE;
            regagcc = RF09_AGCC;
            regagcs = RF09_AGCS;
        } else if (transceiver == Transceiver::RF24) {
            regrxbwc = RF24_RXBWC;
            regrxdfe = RF24_RXDFE;
            regagcc = RF24_AGCC;
            regagcs = RF24_AGCS;
        }

        /// Set RFn_RXBWC
        reg = spi_read_8(regrxbwc, err);
        reg = (reg & 0xC0) | (static_cast<uint8_t>(if_inversion) << 5) |
              (static_cast<uint8_t>(if_shift) << 4) |
              static_cast<uint8_t>(rx_bw);
        spi_write_8(regrxbwc, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// Set RFn_RXDFE
        reg = spi_read_8(regrxdfe, err);
        reg = (reg & 0x10) | (static_cast<uint8_t>(rx_rel_cutoff) << 5) | static_cast<uint8_t>(rx_sample_rate);
        spi_write_8(regrxdfe, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        /// Set RFn_AGGC
        reg = spi_read_8(regagcc, err);
        reg = (reg & (0x1 << 7)) | (static_cast<uint8_t>(agc_input) << 6) | (static_cast<uint8_t>(agc_avg_sample) << 4) | (static_cast<uint8_t>(agc_reset) << 3) | (static_cast<uint8_t>(agc_freeze_control) << 1) | (static_cast<uint8_t>(agc_enable) << 0);
        spi_write_8(regagcc, reg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// Set RFn_AGCS
        if (agc_enable == AGCEnable::agc_disabled)
            reg = (static_cast<uint8_t>(agc_target) << 5) | gain_control_word;
        else {
            // Leave bits [4:0] unchanged, update bits [7:5]
            reg &= 0x1F;                                    // Mask out bits [7:5], keep [4:0] unchanged (0x1F = 00011111) because this value indicated the current receiver gain setting
            reg |= (static_cast<uint8_t>(agc_target) << 5); // Write to bits [7:5]
        }
        spi_write_8(regagcs, reg, err);
    }

    void At86rf215_Utilities::setup_irq_cfg(bool maskMode, IRQPolarity polarity,
                                  PadDriverStrength padDriverStrength, Error& err) {
        RegisterAddress regcfg = RF_CFG;
        uint8_t reg_value = spi_read_8(regcfg, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        reg_value &= ~(0xF);
        reg_value |= (maskMode << 3) | (static_cast<uint8_t>(polarity) << 2) | static_cast<uint8_t>(padDriverStrength);
        spi_write_8(regcfg, reg_value, err);
    }

    void At86rf215_Utilities::setup_phy_baseband(Transceiver transceiver, bool continuousTransmit,
                                       bool frameSeqFilter, bool transmitterAutoFCS,
                                       FrameCheckSequenceType fcsType, bool basebandEnable,
                                       PhysicalLayerType phyType, Error& err) {
        RegisterAddress regphy;

        if (transceiver == Transceiver::RF09) {
            regphy = BBC0_PC;
        } else if (transceiver == Transceiver::RF24) {
            regphy = BBC1_PC;
        }

        spi_write_8(regphy,
                    (continuousTransmit << 7) | (frameSeqFilter << 6) | (transmitterAutoFCS << 4) | (static_cast<uint8_t>(fcsType) << 3) | (basebandEnable << 2) | static_cast<uint8_t>(phyType),
                    err);
    }

    void At86rf215_Utilities::setup_irq_mask(Transceiver transceiver, bool iqIfSynchronizationFailure, bool transceiverError,
                                   bool batteryLow, bool energyDetectionCompletion, bool transceiverReady, bool wakeup,
                                   bool frameBufferLevelIndication, bool agcRelease, bool agcHold,
                                   bool transmitterFrameEnd, bool receiverExtendedMatch, bool receiverAddressMatch,
                                   bool receiverFrameEnd, bool receiverFrameStart, Error& err) {
        RegisterAddress regbbc;
        RegisterAddress regrf;

        if (transceiver == Transceiver::RF09) {
            regbbc = BBC0_IRQM;
            regrf = RF09_IRQM;
        } else if (transceiver == Transceiver::RF24) {
            regbbc = BBC1_IRQM;
            regrf = RF24_IRQM;
        }

        spi_write_8(regrf, iqIfSynchronizationFailure << 5 | transceiverError << 4 | batteryLow << 3 | energyDetectionCompletion << 2 | transceiverReady << 1 | wakeup, err);

        spi_write_8(regbbc,
                    frameBufferLevelIndication << 7 | agcRelease << 6 | agcHold << 5 | transmitterFrameEnd << 4 | receiverExtendedMatch << 3 | receiverAddressMatch << 2 | receiverFrameEnd << 1 | receiverFrameStart, err);
    }

    void At86rf215_Utilities::setup(Error& err) {
        // Check state of RF09 core
        State state = get_state_private(Transceiver::RF09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // We have access to all registers only if we are in the state TRXOFF
        if (state != State::RF_TRXOFF) {
            err = Error::INVALID_STATE_FOR_OPERATION;
            return;
        }
        // Check state of RF24 core - we only proceed with the set-up if both cores are in the TRXOFF state to avoid setting half the registers
        state = get_state_private(Transceiver::RF24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        if (state != State::RF_TRXOFF) {
            err = Error::INVALID_STATE_FOR_OPERATION;
            return;
        }

        // Set IRQ masks
        setup_irq_mask(Transceiver::RF09, radioInterruptsConfig.iqIfSynchronizationFailure09, radioInterruptsConfig.transceiverError09,
                       radioInterruptsConfig.batteryLow09, radioInterruptsConfig.energyDetectionCompletion09, radioInterruptsConfig.transceiverReady09,
                       radioInterruptsConfig.wakeup09, basebandCoreInterruptsConfig.frameBufferLevelIndication09, basebandCoreInterruptsConfig.agcRelease09,
                       basebandCoreInterruptsConfig.agcHold09, basebandCoreInterruptsConfig.transmitterFrameEnd09, basebandCoreInterruptsConfig.receiverExtendedMatch09,
                       basebandCoreInterruptsConfig.receiverAddressMatch09, basebandCoreInterruptsConfig.receiverFrameEnd09, basebandCoreInterruptsConfig.receiverFrameStart09, err);

        setup_irq_mask(Transceiver::RF24, radioInterruptsConfig.iqIfSynchronizationFailure24, radioInterruptsConfig.transceiverError24,
                       radioInterruptsConfig.batteryLow24, radioInterruptsConfig.energyDetectionCompletion24, radioInterruptsConfig.transceiverReady24,
                       radioInterruptsConfig.wakeup24, basebandCoreInterruptsConfig.frameBufferLevelIndication24, basebandCoreInterruptsConfig.agcRelease24,
                       basebandCoreInterruptsConfig.agcHold24, basebandCoreInterruptsConfig.transmitterFrameEnd24, basebandCoreInterruptsConfig.receiverExtendedMatch24,
                       basebandCoreInterruptsConfig.receiverAddressMatch24, basebandCoreInterruptsConfig.receiverFrameEnd24, basebandCoreInterruptsConfig.receiverFrameStart24, err);

        // Set IRQ pin
        setup_irq_cfg(generalConfig.irqMaskMode, generalConfig.irqPolarity,
                      generalConfig.padDriverStrength, err);

        // Set PLL
        configure_pll(Transceiver::RF09, freqSynthesizerConfig, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        configure_pll(Transceiver::RF24, freqSynthesizerConfig, err);
        if (err != Error::NO_ERRORS) {
            return;
        }


        // Setup Physical Layer for Baseband Cores
        setup_phy_baseband(Transceiver::RF09, basebandCoreConfig.continuousTransmit09,
                           basebandCoreConfig.frameCheckSequenceFilterEn09, basebandCoreConfig.transmitterAutoFrameCheckSequence09,
                           basebandCoreConfig.frameCheckSequenceType09, basebandCoreConfig.baseBandEnable09,
                           basebandCoreConfig.physicalLayerType09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        setup_phy_baseband(Transceiver::RF24, basebandCoreConfig.continuousTransmit24,
                           basebandCoreConfig.frameCheckSequenceFilterEn24, basebandCoreConfig.transmitterAutoFrameCheckSequence24,
                           basebandCoreConfig.frameCheckSequenceType24, basebandCoreConfig.baseBandEnable24,
                           basebandCoreConfig.physicalLayerType24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBCn_FSKC0
        set_bbc_fskc0_config(RF09, basebandCoreConfig.bandwidth_time_09, basebandCoreConfig.midxs_09, basebandCoreConfig.midx_09, basebandCoreConfig.mord_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskc0_config(RF24, basebandCoreConfig.bandwidth_time_24, basebandCoreConfig.midxs_24, basebandCoreConfig.midx_24, basebandCoreConfig.mord_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBCn_FSKC1
        set_bbc_fskc1_config(RF09, basebandCoreConfig.freq_inv_09, basebandCoreConfig.sr_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskc1_config(RF24, basebandCoreConfig.freq_inv_24, basebandCoreConfig.sr_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBCn_FSKC2
        set_bbc_fskc2_config(RF09, basebandCoreConfig.preamble_detection_09, basebandCoreConfig.receiver_override_09, basebandCoreConfig.receiver_preamble_timeout_09, basebandCoreConfig.mode_switch_en_09, basebandCoreConfig.preamble_inversion_09, basebandCoreConfig.fec_scheme_09, basebandCoreConfig.interleaving_enable_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskc2_config(RF24, basebandCoreConfig.preamble_detection_24, basebandCoreConfig.receiver_override_24, basebandCoreConfig.receiver_preamble_timeout_24, basebandCoreConfig.mode_switch_en_24, basebandCoreConfig.preamble_inversion_24, basebandCoreConfig.fec_scheme_24, basebandCoreConfig.interleaving_enable_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBCn_FSKC3
        set_bbc_fskc3_config(RF09, basebandCoreConfig.sfdt_09, basebandCoreConfig.prdt_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskc3_config(RF24, basebandCoreConfig.sfdt_24, basebandCoreConfig.prdt_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBC_FSKC4
        set_bbc_fskc4_config(RF09, basebandCoreConfig.sfdQuantization_09, basebandCoreConfig.sfd32_09, basebandCoreConfig.rawModeReversalBit_09, basebandCoreConfig.csfd1_09, basebandCoreConfig.csfd0_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskc4_config(RF24, basebandCoreConfig.sfdQuantization_24, basebandCoreConfig.sfd32_24, basebandCoreConfig.rawModeReversalBit_24, basebandCoreConfig.csfd1_24, basebandCoreConfig.csfd0_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBCn_FSKPHRTX
        set_bbc_fskphrtx(RF09, basebandCoreConfig.sfdUsed_09, basebandCoreConfig.dataWhitening_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskphrtx(RF24, basebandCoreConfig.sfdUsed_24, basebandCoreConfig.dataWhitening_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        /// BBCn_FSKDM
        set_bbc_fskdm(RF09, basebandCoreConfig.fskPreamphasisEnable_09, basebandCoreConfig.directModEnableFskdm_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        set_bbc_fskdm(RF24, basebandCoreConfig.fskPreamphasisEnable_24, basebandCoreConfig.directModEnableFskdm_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // Set TX front-end
        setup_tx_frontend(Transceiver::RF09, txConfig.powerAmplifierRampTime09,
                          txConfig.transmitterCutOffFrequency09,
                          txConfig.txRelativeCutoffFrequency09, txConfig.directModulation09,
                          txConfig.transceiverSampleRate09,
                          txConfig.powerAmplifierCurrentControl09, txConfig.txOutPower09,
                          externalFrontEndConfig.externalLNABypass09, externalFrontEndConfig.automaticGainControlMAP09,
                          externalFrontEndConfig.automaticVoltageExternal09, externalFrontEndConfig.analogVoltageEnable09,
                          externalFrontEndConfig.powerAmplifierVoltageControl09, externalFrontEndConfig.externalFrontEnd_09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        setup_tx_frontend(Transceiver::RF24, txConfig.powerAmplifierRampTime24,
                          txConfig.transmitterCutOffFrequency24,
                          txConfig.txRelativeCutoffFrequency24, txConfig.directModulation24,
                          txConfig.transceiverSampleRate24,
                          txConfig.powerAmplifierCurrentControl24, txConfig.txOutPower24,
                          externalFrontEndConfig.externalLNABypass24, externalFrontEndConfig.automaticGainControlMAP24,
                          externalFrontEndConfig.automaticVoltageExternal24, externalFrontEndConfig.analogVoltageEnable24,
                          externalFrontEndConfig.powerAmplifierVoltageControl24, externalFrontEndConfig.externalFrontEnd_24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        // Set up RX front-end
        setup_rx_frontend(Transceiver::RF09, rxConfig.ifInversion09, rxConfig.ifShift09,
                          rxConfig.receiverBandwidth09, rxConfig.rxRelativeCutoffFrequency09,
                          rxConfig.receiverSampleRate09, rxConfig.agcInput09,
                          rxConfig.averageTimeNumberSamples09, rxConfig.agcReset_09, rxConfig.agcFreezeControl_09, rxConfig.agcEnabled09,
                          rxConfig.automaticGainTarget09, rxConfig.gainControlWord09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        setup_rx_frontend(Transceiver::RF24, rxConfig.ifInversion24, rxConfig.ifShift24,
                          rxConfig.receiverBandwidth24, rxConfig.rxRelativeCutoffFrequency24,
                          rxConfig.receiverSampleRate24, rxConfig.agcInput24,
                          rxConfig.averageTimeNumberSamples24, rxConfig.agcReset_24, rxConfig.agcFreezeControl_24, rxConfig.agcEnabled24,
                          rxConfig.automaticGainTarget24, rxConfig.gainControlWord24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        // Set up IQ interface
        setup_iq(iqInterfaceConfig.externalLoopback, iqInterfaceConfig.iqOutputCurrent,
                 iqInterfaceConfig.iqmodeVoltage, iqInterfaceConfig.iqmodeVoltageIEE,
                 iqInterfaceConfig.embeddedControlTX, iqInterfaceConfig.chipMode, iqInterfaceConfig.skewAlignment,
                 err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        /// Set up energy detection
        /// RFn_EDC, RFn_EDD
        setup_rx_energy_detection(Transceiver::RF09, rxConfig.energyDetectionMode09,
                                  rxConfig.energyDetectDurationFactor09, rxConfig.energyDetectionBasis09, err);
        if (err != Error::NO_ERRORS) {
            return;
        }
        setup_rx_energy_detection(Transceiver::RF24, rxConfig.energyDetectionMode24,
                                  rxConfig.energyDetectDurationFactor24, rxConfig.energyDetectionBasis24, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        /// Set up battery
        /// RF_BMDVC
        set_battery_monitor_control(generalConfig.batteryMonitorHighRange, generalConfig.batteryMonitorVoltage, err);
        if (err != Error::NO_ERRORS) {
            return;
        }

        /// Set up crystal oscillator
        /// RF_XOC
        setup_crystal(generalConfig.fastStartUp, generalConfig.crystalTrim, err);
    }

    uint8_t At86rf215_Utilities::get_irq(Transceiver transceiver, Error& err) {
        if (transceiver == RF09) {
            return spi_read_8(RF09_IRQS, err);
        } else { // transceiver == RF24
            return spi_read_8(RF24_IRQS, err);
        }
        return 0;
    }

    void At86rf215_Utilities::set_bbc_fskc0_config(Transceiver transceiver,
                                         Bandwidth_time_product bt, Mod_index_scale midxs, Mod_index midx, FSK_mod_order mord,
                                         Error& err) {
        // Define the appropriate register for BBCn_FSKC0 based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKC0; // Replace with actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKC0; // Replace with actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        reg_value &= 0x00; // Clear the bits that will be set explicitly

        // Clear existing values in BT, MIDXS, MIDX, and MORD
        reg_value |= ((static_cast<uint8_t>(bt) & 0x03) << 6);    // BT: Bits [7:6]
        reg_value |= ((static_cast<uint8_t>(midxs) & 0x03) << 4); // MIDXS: Bits [5:4]
        reg_value |= ((static_cast<uint8_t>(midx) & 0x07) << 1);  // MIDX: Bits [3:1]
        reg_value |= (static_cast<uint8_t>(mord) & 0x01);         // MORD: Bit [0]

        // Write the updated value back to the register
        spi_write_8(reg_address, reg_value, err);
    }
    void At86rf215_Utilities::set_bbc_fskc1_config(Transceiver transceiver,
                                         Freq_Inversion freq_inv, MR_FSK_symbol_rate sr,
                                         Error& err) {
        // Define the appropriate register for BBCn_FSKC1 based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKC1; // Replace with actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKC1; // Replace with actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        // clear all bits except bit 4 (counting from 4)
        reg_value &= (0x1 << 4);
        reg_value |= (static_cast<uint8_t>(freq_inv) & 0x1) << 5;
        reg_value |= (static_cast<uint8_t>(sr) & 0xF);
        // Write the updated value back to the register
        spi_write_8(reg_address, reg_value, err);
    }
    void At86rf215_Utilities::set_bbc_fskc2_config(Transceiver transceiver, Preamble_Detection preamble_det,
                                         Receiver_Override rec_override,
                                         Receiver_Preamble_Timeout rec_preamble_timeout,
                                         Mode_Switch_Enable mode_switch_en,
                                         Preamble_Inversion preamble_inversion,
                                         FEC_Scheme fec_scheme,
                                         Interleaving_Enable interleaving_enable, Error& err) {
        // Define the appropriate register for BBCn_FSKC2 based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKC2; // Replace with actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKC2; // Replace with actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        // Update the register value with provided configurations
        reg_value &= 0x00; // Clear the bits that will be set explicitly
        // Bit 7: PDTM - Preamble Detection Mode
        reg_value |= (static_cast<uint8_t>(preamble_det) & 0x1) << 7;
        // Bits 6-5: RXO - Receiver Override
        reg_value |= (static_cast<uint8_t>(rec_override) & 0x3) << 5;
        // Bit 4: RXPTO - Receiver Preamble Time Out
        reg_value |= (static_cast<uint8_t>(rec_preamble_timeout) & 0x1) << 4;
        // Bit 3: MSE - Mode Switch Enable
        reg_value |= (static_cast<uint8_t>(mode_switch_en) & 0x1) << 3;
        // Bit 2: PRI - Preamble Inversion
        reg_value |= (static_cast<uint8_t>(preamble_inversion) & 0x1) << 2;
        // Bit 1: FECS - FEC Scheme
        reg_value |= (static_cast<uint8_t>(fec_scheme) & 0x1) << 1;
        // Bit 0: FECIE - Interleaving Enable
        reg_value |= (static_cast<uint8_t>(interleaving_enable) & 0x1) << 0;
        // Write the updated value back to the register
        spi_write_8(reg_address, reg_value, err);
    }

    void At86rf215_Utilities::set_bbc_fskc3_config(Transceiver transceiver, SFD_Detection_Threshold sfdDetectionThreshold, Preamble_Detection_Threshold preambleDetectionThreshold, Error& err) {
        // Define the appropriate register for BBCn_FSKC2 based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKC3; // Replace with actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKC3; // Replace with actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        // Update the register value with provided configurations
        reg_value &= 0x00; // Clear the bits that will be set explicitly
        reg_value |= (static_cast<uint8_t>(sfdDetectionThreshold) & 0xF) << 4;
        reg_value |= (static_cast<uint8_t>(preambleDetectionThreshold) & 0xF) << 0;
        // Write the updated value back to the register
        spi_write_8(reg_address, reg_value, err);
    }

    void At86rf215_Utilities::set_bbc_fskc4_config(Transceiver transceiver,
                                         SFD_Quantization sfd_quantization,
                                         SFD_32 sfd_32,
                                         Raw_Mode_Reversal_Bit raw_mode_reversal,
                                         CSFD1 csfd1,
                                         CSFD0 csfd0,
                                         Error& err) {
        // Define the appropriate register address for BBCn_FSKC4 based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKC4; // Replace with the actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKC4; // Replace with the actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        // clear all bits except bit 7
        reg_value &= (0x01) << 7;
        reg_value |= (static_cast<uint8_t>(sfd_quantization) & 0x1) << 6;
        reg_value |= (static_cast<uint8_t>(sfd_32) & 0x1) << 5;
        reg_value |= (static_cast<uint8_t>(raw_mode_reversal) & 0x1) << 4;
        reg_value |= (static_cast<uint8_t>(csfd1) & 0x3) << 2;
        reg_value |= (static_cast<uint8_t>(csfd0) & 0x3) << 0;
        // Write the updated value back to the register
        spi_write_8(reg_address, reg_value, err);
    }
    void At86rf215_Utilities::set_bbc_fskphrtx(Transceiver transceiver, SFD_Used sfdUsed, Data_Whitening dataWhitening, Error& err) {
        // Define the appropriate register address for BBC0_FSKPHRTX based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKPHRTX; // Replace with the actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKPHRTX; // Replace with the actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        // clear the bits to be updated
        // 0000 1000 | 0000 0100 = 0000 1100 -> 1111 0011 -> reg_value = reg_value & 1111 0011
        reg_value &= ~((0x1 << 3) | (0x1 << 2));
        reg_value |= (static_cast<uint8_t>(sfdUsed) & 0x1) << 3;
        reg_value |= (static_cast<uint8_t>(dataWhitening) & 0x1) << 2;
        spi_write_8(reg_address, reg_value, err);
    }

    void At86rf215_Utilities::set_bbc_fskdm(Transceiver transceiver, FSK_Preamphasis_Enable fskPreamphasisEnable, Direct_Mod_Enable_FSKDM directModEnableFskdm, Error& err) {
        // Define the appropriate register address for BBCn_FSKDM based on the transceiver
        RegisterAddress reg_address;
        if (transceiver == RF09) {
            reg_address = BBC0_FSKDM; // Replace with the actual RF09 register address
        } else { // transceiver == RF24
            reg_address = BBC1_FSKDM; // Replace with the actual RF24 register address
        }

        // Read the current register value and mask out the fields to preserve other bits
        uint8_t reg_value = spi_read_8(reg_address, err);
        if (err != Error::NO_ERRORS) {
            return; // Return early if SPI read fails
        }
        // clear bits [1:0]
        reg_value &= ~((0x01 << 1) | (0x01 << 0));
        reg_value |= (static_cast<uint8_t>(fskPreamphasisEnable) & 0x1) << 1;
        reg_value |= (static_cast<uint8_t>(directModEnableFskdm) & 0x1) << 0;
        spi_write_8(reg_address, reg_value, err);
    }

    etl::expected<uint16_t, Error> At86rf215_Utilities::get_received_length(Transceiver transceiver, Error& err) {
        RegisterAddress reg_address_low;
        RegisterAddress reg_address_high;

        // Determine the appropriate register addresses based on the transceiver
        if (transceiver == RF09) {
            reg_address_low = BBC0_RXFLL; // Replace with actual RF09 register address
            reg_address_high = BBC0_RXFLH;
        } else {
            reg_address_low = BBC1_RXFLL; // Replace with actual RF24 register address
            reg_address_high = BBC1_RXFLH;
        }
        uint8_t low_length_byte = spi_read_8(reg_address_low, err);
        if (err != Error::NO_ERRORS) {
            return etl::unexpected<Error>(err); // Return the error
        }

        // Read the high-length byte
        uint8_t high_length_byte = spi_read_8(reg_address_high, err);
        if (err != Error::NO_ERRORS) {
            return etl::unexpected<Error>(err); // Return the error
        }
        // Combine the bytes to form the received length
        uint16_t received_length = (static_cast<uint16_t>(high_length_byte) << 8) | low_length_byte;
        return received_length;
    }

    At86rf215_Utilities transceiverUtils = At86rf215_Utilities();
} // namespace AT86RF215
/**
 * @file at86rf215GuardUtilities.hpp
 *
 * @breif This file contains RAII like objects, which are used for:
 * - locking/unlocking mutexes protecting different parts of the transceiver
 * - preparing the transceiver for a specific operating mode and restoring to the original configuration
 *
 * @note These objects are not RAII by strict definition, as the mutex locking (or transceiver setup)
 *       is not performed in the constructor, but in methods. This design choice stems from the
 *       fact that exception are forbidden, so it would not have been easy to know if the operation inside the
 *       constructor succeeded or failed. Of course mutex unlocking (or reversal of transceiver setup) is
 *       performed in the destructor.
 *
 * @note It is useful to know that the C++ standard guarantees that objects are destroyed in the reverse order
 *       they are initialized, as explained here:
 *       https://isocpp.org/wiki/faq/dtors
 *
 */

#pragma once
#include "FreeRTOS.h"
#include "semphr.h"
#include "at86rf215.hpp"
#include "at86rf215Definitions.hpp"

namespace AT86RF215 {
    /**
     * Mutex lock guard. To avoid deadlocks, the locking order must strictly
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
    class AT86RF215Chip::MutexGuard {
    public:
        MutexGuard(AT86RF215Chip& chip) : chip(chip) {}

        MutexGuard(const MutexGuard&) = delete;
        MutexGuard& operator=(const MutexGuard&) = delete;
        ~MutexGuard();

        bool lockSpi();

        void unlockSpi();

        bool lockIqTx();

        void unlockIqTx();

        bool lockTransceiver(Transceiver transceiver);

        bool lockAll();

    private:
        AT86RF215Chip& chip;

        bool ownsSpi = false;
        bool ownsRf09 = false;
        bool ownsRf24 = false;
        bool ownsIqTx = false;
    };

    /**
     * Configure a transceiver in a way such that the I/Q DACs are outputting a constant amplitude (maximum for the I
     * DAC and zero for the Q DAC). The procedure follows table 13-2
     *
     * @note It is assumed that the function that creates a DacOverrideSetup instance, already has gained access
     *       to the required resources via a MutexGuard instance
     */
    class AT86RF215Chip::DacOverrideSetup {
    public:
        DacOverrideSetup(AT86RF215Chip& chip, Transceiver transceiver)
        : chip(chip), transceiver(transceiver), iqfc0Reg(RegisterAddress::RF_IQIFC0) {
            if (transceiver == Transceiver::RF09) {
                pcReg = RegisterAddress::BBC0_PC;
                txfhlReg = RegisterAddress::BBC0_TXFLH;
                txfllReg = RegisterAddress::BBC0_TXFLL;
                txdaciReg = RegisterAddress::RF09_TXDACI;
                txdacqReg = RegisterAddress::RF09_TXDACQ;
            } else {
                pcReg = RegisterAddress::BBC1_PC;
                txfhlReg = RegisterAddress::BBC1_TXFLH;
                txfllReg = RegisterAddress::BBC1_TXFLL;
                txdaciReg = RegisterAddress::RF24_TXDACI;
                txdacqReg = RegisterAddress::RF24_TXDACQ;
            }
        }

        DacOverrideSetup(const DacOverrideSetup&) = delete;
        DacOverrideSetup& operator=(const DacOverrideSetup&) = delete;
        ~DacOverrideSetup();

        etl::expected<void, Error> setup();
    private:
        AT86RF215Chip& chip;
        Transceiver transceiver;

        RegisterAddress iqfc0Reg;
        RegisterAddress pcReg;
        RegisterAddress txfhlReg;
        RegisterAddress txfllReg;
        RegisterAddress txdaciReg;
        RegisterAddress txdacqReg;

        etl::optional<uint8_t> iqfc0ValInitial;
        etl::optional<uint8_t> pcValInitial;
        etl::optional<uint8_t> txdaciInitial;
        etl::optional<uint8_t> txdacqInitial;
    };

    /**
     * Ensure that a transceiver has the desirable bandwidth and that the baseband core is turned off, so that a
     * single shot energy measurement may be performed
     *
     * @note It is assumed that the function that creates a SingleShotMeasurementSetup instance, already has gained access
     *       to the required resources via a MutexGuard instance
     */
    class AT86RF215Chip::SingleShotMeasurementSetup {
    public:
        SingleShotMeasurementSetup(AT86RF215Chip& chip, Transceiver transceiver, etl::optional<ReceiverBandwidth> bw)
        : chip(chip), transceiver(transceiver), bw(bw) {
            if (transceiver == Transceiver::RF09) {
                rxbwcReg = RegisterAddress::RF09_RXBWC;
                bbcPcReg = RegisterAddress::BBC0_PC;
            } else {
                rxbwcReg = RegisterAddress::RF24_RXBWC;
                bbcPcReg = RegisterAddress::BBC1_PC;
            }
        }

        SingleShotMeasurementSetup(const SingleShotMeasurementSetup&) = delete;
        SingleShotMeasurementSetup& operator=(const SingleShotMeasurementSetup&) = delete;
        ~SingleShotMeasurementSetup();

        etl::expected<void, Error> setup();
    private:
        AT86RF215Chip& chip;
        Transceiver transceiver;
        etl::optional<ReceiverBandwidth> bw;

        RegisterAddress rxbwcReg;
        RegisterAddress bbcPcReg;

        etl::optional<uint8_t> rxbwcInitial;
        etl::optional<uint8_t> bbcPcInitial;
    };

    /**
     * Ensure that the requested transceiver has the baseband core enabled
     *
     * @note It is assumed that the function that creates a SingleShotMeasurementSetup instance, already has gained access
     *       to the required resources via a MutexGuard instance
     */
    class AT86RF215Chip::IntBasebandCoreBasicModeSetup {
    public:
        IntBasebandCoreBasicModeSetup(AT86RF215Chip& chip, Transceiver transceiver)
        : chip(chip), transceiver(transceiver) {
            if (transceiver == Transceiver::RF09) {
                pcReg = RegisterAddress::BBC0_PC;
            } else {
                pcReg = RegisterAddress::BBC1_PC;
            }
        }

        IntBasebandCoreBasicModeSetup(const IntBasebandCoreBasicModeSetup&) = delete;
        IntBasebandCoreBasicModeSetup& operator=(const IntBasebandCoreBasicModeSetup&) = delete;
        ~IntBasebandCoreBasicModeSetup();

        etl::expected<void, Error> setup();
    private:
        AT86RF215Chip& chip;
        Transceiver transceiver;

        RegisterAddress pcReg;

        etl::optional<uint8_t> pcInitial;
    };
} // namespace AT86RF215

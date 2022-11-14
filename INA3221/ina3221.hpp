#ifndef COMPONENT_DRIVERS_INA3221_HPP
#define COMPONENT_DRIVERS_INA3221_HPP

#include <cstdint>
#include <valarray>
#include <optional>
#include <type_traits>
#include <tuple>

namespace INA3221 {
    typedef std::pair<uint32_t, uint32_t> VoltageThreshold;

    typedef std::tuple<std::optional<uint32_t>, std::optional<uint32_t>, std::optional<uint32_t>> ChannelMeasurement;

    /// Register address
    enum class Register {
        /// Configuration
        CONFG = 0x00,
        /// Channel 1 Shunt Voltage
        CH1SV = 0x01,
        /// Chanel 1 Bus Voltage
        CH1BV = 0x02,
        /// Channel 2 Shunt Voltage
        CH2SV = 0x03,
        /// Chanel 2 Bus Voltage
        CH2BV = 0x04,
        /// Channel 3 Shunt Voltage
        CH3SV = 0x05,
        /// Chanel 3 Bus Voltage
        CH3BV = 0x06,
        /// Chanel 1 Critical Alert Limit
        CH1CA = 0x07,
        /// Chanel 1 Warning Alert Limit
        CH1WA = 0x08,
        /// Chanel 2 Critical Alert Limit
        CH2CA = 0x09,
        /// Chanel 2 Warning Alert Limit
        CH2WA = 0x0A,
        /// Chanel 3 Critical Alert Limit
        CH3CA = 0x0B,
        /// Chanel 3 Warning Alert Limit
        CH3WA = 0x0C,
        /// Shunt-Voltage Sum
        SHVOL = 0x0D,
        /// Shunt-Voltage Sum Limit
        SHVLL = 0x0E,
        /// Mask-Enable
        MASKE = 0x0F,
        /// Power-Valid Upper Limit
        PWRVU = 0x10,
        /// Power-Valid Lower Limit
        PWRVL = 0x11,
        /// Manufacturer ID
        MFRID = 0xFE,
        /// Die ID
        DIEID = 0xFF,
    };

    /// Alert level
    enum class Alert {
        CRITICAL = 0,
        WARNING,
        POWER_VALID,
        TIMING_CONTROL
    };

    /// Error status
    enum class Error {
        NO_ERRORS = 0,
        I2C_FAIlURE,
        INVALID_STATE,
    };

    /// The number of voltage samples that are averaged together
    enum class AveragingMode {
        AVG_1 = 0,
        AVG_4 = 1,
        AVG_16 = 2,
        AVG_64 = 3,
        AVG_128 = 4,
        AVG_256 = 5,
        AVG_512 = 6,
        AVG_1024 = 7,
    };

    /// Bus voltage conversion time
    enum class VoltageTime {
        /// 140 μs
        V_140_MS = 0,
        /// 240 μs
        V_204_MS = 1,
        /// 322 μs
        V_332_MS = 2,
        /// 588 μs
        V_588_MS = 3,
        /// 1.1 μs
        V_1_1_MS = 4,
        /// 2.116 μs
        V_2_116_MS = 5,
        /// 4.156 μs
        V_4_156_MS = 6,
        /// 8.244 μs
        V_8_244_MS = 7,
    };

    /**
     * Operating mode of INNA3211. The main three modes are:
     *  - Power down: Turns off the current drawn to reduce power consumption. Switching from power-down
     *  mode takes approximately 40 μs. I2C communication is still enabled while in this mode
     *  - Single shot: Measurements are taken only whenever this register is written and set to single shot mode
     *  (there's no need to switch the value from a previous different state).
     *  - Continuous: Measurements are constantly taken periodically until the mode is switched to either
     *  single-shot or power down
     *
     *  This register also controls whether this is applies only to shunt voltage, bus voltage
     *  or both.
     */
    enum class OperatingMode {
        POWER_DOWN = 0,               /// Power-down
        SHUNT_VOLTAGE_SS = 1,         /// Shunt Voltage, Single-Shot
        BUS_VOLTAGE_SS = 2,           /// Bus Voltage, Single-Shot
        SHUNT_BUS_VOLTAGE_SS = 3,     /// Shunt & Bus Voltage, Single-Shot
        POWER_DOWN_REND = 4,          /// Power-down
        SHUNT_VOLTAGE_CONT = 5,       /// Shunt Voltage, Continuous
        BUS_VOLTAGE_CONT = 6,         /// Bus Voltage, Continuous
        SHUNT_BUS_VOLTAGE_CONT = 7,   /// Shunt & Bus Voltage, Continuous
    };

    /**
     * Config of INA3221. It sets up the necessary values upon powering the device
     */
    struct INA3221Config {

        /// Determine whether summation channel value is periodically updated
        bool summationChannelControl1 = true;
        bool summationChannelControl2 = true;
        bool summationChannelControl3 = true;

        /// Warning alerts enabled
        bool enableWarnings = true;

        /// Critical alerts enabled
        bool enableCritical = true;

        /// Determines whether channel 1 is enabled
        bool enableChannel1 = true;

        /// Determines whether channel 2 is enabled
        bool enableChannel2 = true;

        /// Determines whether channel 3 is enabled
        bool enableChannel3 = true;

        /// The number of voltage samples that are averaged together
        AveragingMode averagingMode = AveragingMode::AVG_4;

        /**
          * Time of bus voltage measurement conversion.
          * This value should be selected according to the time requirements of the application.
          * Note that it applies to all channels
         */
        VoltageTime busVoltageTime = VoltageTime::V_1_1_MS;

        /**
          * Time of shunt voltage measurement conversion.
          * This value should be selected according to the time requirements of the application.
          * Note that it applies to all channels
         */
        VoltageTime shuntVoltageTime = VoltageTime::V_1_1_MS;

        /**
         * Select mode operation of INNA3211. The main three modes are:
         *  - Power down: Turns off the current drawn to reduce power consumption. Switching from power-down
         *  mode takes approximately 40 μs. I2C communication is still enabled while in this mode
         *  - Single shot: Measurements are taken only whenever this register is written and set to single shot mode
         *  (there's no need to switch the value from a previous different state).
         *  - Continuous: Measurements are constantly taken periodically until the mode is switched to either
         *  single-shot or power down
         *
         *  This register also controls whether this is applies only to shunt voltage, bus voltage
         *  or both.
         */
        OperatingMode operatingMode = OperatingMode::POWER_DOWN;

        /// Shunt voltage threshold for critical and warning alert for channel1 [μV]
        VoltageThreshold threshold1;

        /// Shunt voltage threshold for critical and warning alert for channel2 [μV]
        VoltageThreshold threshold2;

        /// Shunt voltage threshold for critical and warning alert for channel3 [μV]
        VoltageThreshold threshold3;

        /// Shunt voltage sum limit [μV]
        uint32_t shuntVoltageSumLimit;

        /// Upper limit of power-valid [μV]
        uint32_t powerValidUpper;

        /// Lower limit of power-valid [μV]
        uint32_t powerValidLower;
    };

    class INA3221 {
    public:
        INA3221(const INA3221Config &&config, Error &err) :
                config(std::move(config)) {
            setup();
        };

        ~INA3221() {};

    private:
        // TODO: Replace with HAL delay
        __attribute__ ((__weak__)) void wait(uint16_t msec);

        /// TODO: Replace with platform-specific I2C write request
        /**
         * Writes two bytes in the given register via I2C
         *
         * @param address       Register address
         * @param value         16-bit value to write to
         * @return              Error status
         */
        [[nodiscard]] __attribute__ ((__weak__)) Error i2c_write(Register address, uint16_t value);

        /// TODO: Replace with platform-specific I2C read request
        /**
         * Reads a given 16-bit register via I2C
         *
         * @param address       Register address
         * @return              read value and error status
         */
        [[nodiscard]] __attribute__ ((__weak__)) std::pair<uint16_t, Error> i2c_read(Register address) {
            return std::make_pair<uint16_t, Error>(0, Error::NO_ERRORS);
        };

        /**
         * Writes to a specific field of the register
         * @param address       Register address
         * @param value         Value to write to
         * @param mask          Mask of the value
         * @param shift         Shift bits - determines the register field to write to
         * @return              Error status
         */
        [[nodiscard]] Error write_register_field(Register address, uint16_t value, uint16_t mask, uint16_t shift);

        /**
         * Sets the config register as per the passed config
         * @return Raised error
         */
        [[nodiscard]] Error setup();

        /**
         * Triggers a measurement of bus or shunt voltage for the active channels. Note that the driver halts until we
         * get a valid reading or an alert is raised.
         *
         * If the mode is set to continuous, values will be updated periodically depending on the samples and conversion
         * time set. For single-shot mode
         */
        [[nodiscard]] Error take_measurement();

        /// Bus voltage across the three measured channels (NULL values indicate that the channel isn't currently monitored)
        ChannelMeasurement busVoltage{NULL, NULL, NULL};
        /// Shunt voltage across the three measured channels (NULL values indicate that the channel isn't currently monitored)
        ChannelMeasurement shuntVoltage{NULL, NULL, NULL};

        void handle_irq(void);

        INA3221Config config;
    };
}


#endif //COMPONENT_DRIVERS_INA3221_HPP
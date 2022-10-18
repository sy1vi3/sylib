/**
 * \file sylib/motor.hpp
 *
 * \brief Contains prototypes for functions relating to smart motor
 * control and telemetry
 */

#pragma once
#include "env.hpp"
#include "sylib.hpp"
#include "sylib_apitypes.hpp"
#include "system.hpp"

namespace sylib {

/**
 * \brief Accurate Motor Velocity Estimator
 */
class SylviesPogVelocityEstimator {
   private:
    uint32_t previousInternalMotorClock;
    double oldMotorTicks;
    double dN;
    double dT;
    double rawVelocity;
    double motorVoltageTarget;
    double smaFilteredVelocity;
    double medianFilteredVelocity;
    double emaFilteredVelocity;
    double smaFilteredAccel;
    double smaFilteredJerk;
    double smaFilteredSnap;
    double motorGearing;
    double outputVelocity;
    double preFilterAcceleration;
    double preFilterJerk;
    double preFilterSnap;
    double outputAcceleration;
    double outputJerk;
    double outputSnap;
    double accelCalculatedkA;
    double maxFilteredAccel;
    sylib::SMAFilter smaFilterVelocity;
    sylib::SMAFilter smaFilterAccel;
    sylib::SMAFilter smaFilterJerk;
    sylib::SMAFilter smaFilterSnap;
    sylib::EMAFilter emaFilter;
    sylib::MedianFilter medianFilter;
    sylib::SympleDerivativeSolver preFilterAccelSolver;
    sylib::SympleDerivativeSolver preFilterJerkSolver;
    sylib::SympleDerivativeSolver preFilterSnapSolver;
    sylib::SympleDerivativeSolver outputAccelSolver;
    sylib::SympleDerivativeSolver outputJerkSolver;
    sylib::SympleDerivativeSolver outputSnapSolver;
    sylib::RangeExtremaFilter maxFilterAccel;

   public:
    /**
     * \brief Creates a velocity estimator for a smart motor
     *
     * \param motorGearing
     *        The output speed of the motor, in RPM, if the motor internally
     *        is spinning at 3600 RPM. 200 is default for a green motor cartridge
     */
    SylviesPogVelocityEstimator(double motorGearing = 200);

    /**
     * \brief Calculates motor velocity
     *
     * \param currentMotorTicks
     *        The current number of raw encoder ticks the motor has recorded
     *
     * \return Velocity in RPM
     */
    double getVelocity(double currentMotorTicks, std::uint32_t currentInternalMotorClock);

    /**
     * \brief Gets the most recent calculated motor velocity
     *
     * \return Velocity in RPM
     */
    double getVelocityNoCalculations() const;

    /**
     * \brief Gets the raw motor velocity
     *
     * This value is not filtered, and will be of limitied use.
     * It is purely difference in ticks divided by difference in time.
     *
     * \return Velocity in RPM out of 3600
     */
    double getRawVelocity() const;

    /**
     * \brief Gets the most recent user-provided motor position
     *
     * \return Ticks
     */
    double getRawPosition() const;

    /**
     * \brief Gets the motor velocity put through a 3-tap simple moving average
     *
     * This value is partially filtered, and will be of limitied use.
     *
     * \return Velocity in RPM out of 3600
     */
    double getSmaFilteredVelocity() const;

    /**
     * \brief Gets the motor velocity put through a an exponential moving average filter
     *
     * This value is the same as the estimator's output velocity, but without the motor gearset
     * translation.
     *
     * \return Velocity in RPM out of 3600
     */
    double getEmaFilteredVelocity() const;

    /**
     * \brief Gets the motor velocity put through a median filter
     *
     * This value is partially filtered, and will be of limitied use.
     *
     * \return Velocity in RPM out of 3600
     */
    double getMedianFilteredVelocity() const;

    /**
     * \brief Gets the motor acceleration
     *
     * This value is a second derivative of any actual measurement.
     * It will not be very accurate.
     *
     * \return RPM per millisecond
     */
    double getAcceleration() const;

    /**
     * \brief Gets the raw motor acceleration based on a partially
     * filtered velocity calculation
     *
     * This value is a second derivative of any actual measurement.
     * It will not be very accurate.
     *
     * \return RPM per millisecond
     */
    double getPreFilterAcceleration() const;

    /**
     * \brief Gets the motor jerk
     *
     * This value is a third derivative of any actual measurement.
     * It will not be very accurate.
     *
     * \return RPM per millisecond per millisecond
     */
    double getJerk() const;

    /**
     * \brief Gets the motor snap
     *
     * This value is a fourth derivative of any actual measurement.
     * It will not be very accurate.
     *
     * \return RPM per millisecond per millisecond per millisecond
     */
    double getSnap() const;

    /**
     * \brief Gets the current value of the variable gain used for the
     * velocity EMA filter
     *
     * \return kA value
     */
    double getCalculatedkA() const;

    /**
     * \brief Gets the absolute value of the largest or smallest acceleration
     * measurement in the last 200ms
     *
     * This value is a second derivative of any actual measurement.
     * It will not be very accurate.
     *
     * \return RPM per millisecond
     */
    double getMaxFilteredAcceleration() const;
};

/**
 * \brief V5 Smart Motor
 */
class Motor : private Device {
   private:
    // SDK things
    const V5_DeviceT device;
    void set_motor_controller();
    // Config settings
    const std::uint8_t smart_port;
    const double gearing;
    bool reversed;
    sylib::SylviesPogVelocityEstimator motorVelocityTracker;
    sylib::VoltageEstimation motorVoltageEstimator;
    sylib::ProportionalController motorPController;
    sylib::IntegralController motorIController;
    sylib::DerivativeController motorDController;
    sylib::TakeBackHalfController motorTBHController;
    sylib::SpeedControllerInfo speedController;

    // Updating values
    uint32_t internalClock;
    double velocity;
    double velocity_error;
    double raw_velocity;
    double vexos_velocity;
    double sma_velocity;
    double acceleration;
    double position;
    double temperature;
    double power;
    double target_position;
    double target_velocity;
    std::int32_t current_draw;
    std::int32_t direction;
    std::int32_t efficiency;
    std::int32_t faults;
    std::int32_t flags;
    double torque;
    std::int32_t voltage;
    std::int32_t zero_position_flag;
    std::int32_t stopped;
    std::int32_t over_temp;
    std::int32_t over_current;
    V5MotorBrakeMode brake_mode;
    std::int32_t current_limit;
    std::int32_t voltage_limit;

    // Target values
    SylibMotorControlMode sylib_control_mode;
    double position_target;
    double velocity_target;
    std::int32_t voltage_target;

   public:
    /**
     * \brief Creates a smart motor object
     *
     * \param smart_port
     *        The 1-indexed smart port the motor uses
     *
     * \param gearing
     *        The output speed of the motor, in RPM, if the motor internally
     *        is spinning at 3600 RPM. 200 is default for a green motor cartridge
     *
     * \param reverse
     *        Optional motor reversed flag
     *
     * \param speedController
     *        Optional velocity controller used by set_velocity_custom_controller
     */
    Motor(const uint8_t smart_port, const double gearing = 200, const bool reverse = false,
          const SpeedControllerInfo speedController = SpeedControllerInfo());

    // Background control stuff

    /**
     * \brief Motor update loop
     *
     * Updates the motor to the most recently set values
     *
     * Users do not need to call this function, it is handled
     * by the sylib daemon. Don't use this. It won't help.
     */
    void update() override;

    // Motor control

    /**
     * \brief Sets the absolute motor position target
     *
     * This uses the smart motor's own built-in controller
     *
     * \param new_position
     *        Absolute position to move to
     *
     * \param velocity_cap
     *        Target speed to perform the movement at
     */
    void set_position_target_absolute(double new_position, std::int32_t velocity_cap = 200);

    /**
     * \brief Sets a motor position target relative to its current position
     *
     * This uses the smart motor's own built-in controller
     *
     * \param new_position
     *        Position to move to
     *
     * \param velocity_cap
     *        Target speed to perform the movement at
     */
    void set_position_target_relative(double new_position, std::int32_t velocity_cap = 200);

    /**
     * \brief Sets the motor velocity target and changes control modes
     *
     * This uses a controller provided by the user
     *
     * \param new_velocity_target
     *        Motor velocity target
     */
    void set_velocity_custom_controller(std::int16_t new_velocity_target);

    /**
     * \brief Stops the motor
     */
    void stop();

    /**
     * \brief Sets the voltage sent to the motor directly
     *
     * \param new_voltage_target
     *        Voltage in millivolts
     */
    void set_voltage(std::int16_t new_voltage_target);

    /**
     * \brief Sets the motor braking mode
     *
     * \param mode
     *        Brake mode to use
     */
    void set_braking_mode(V5MotorBrakeMode mode);

    /**
     * \brief Sets the motor amperage limit
     *
     * \param limit
     *        Limit in milliamps
     */
    void set_amps_limit(std::int32_t limit);

    /**
     * \brief Sets the motor reversed flag
     *
     * \param reverse
     *        Reverse flag
     */
    void set_is_reversed(bool reverse);

    /**
     * \brief Sets the motor voltage limit
     *
     * \param limit
     *        Limit in millivolts
     */
    void set_volts_limit(std::int32_t limit);

    /**
     * \brief Tares the motor encoder
     */
    void tare_encoder();

    /**
     * \brief Sets the motor velocity target
     *
     * This uses the smart motor's own built-in controller
     *
     * \param new_velocity_target
     *        Motor velocity target
     */
    void set_velocity(std::int32_t new_velocity_target);

    /**
     * \brief Sets the custom speed controller used by set_velocity_custom_controller
     *
     * \param controller
     *        Velocity controller
     */
    void updateSpeedController(sylib::SpeedControllerInfo controller);

    // Motor telemetry

    /**
     * \brief Gets the current error between the motor's actual velocity and the velocity target
     *
     * Uses the sylib velocity estimator
     *
     * \return RPM
     */
    double get_velocity_error() const;

    /**
     * \brief Gets the current output of the user-provided P controller
     *
     * \return Millivolts
     */
    std::int32_t get_p_voltage() const;

    /**
     * \brief Gets the current output of the user-provided I controller
     *
     * \return Millivolts
     */
    std::int32_t get_i_voltage() const;

    /**
     * \brief Gets the current output of the user-provided D controller
     *
     * \return Millivolts
     */
    std::int32_t get_d_voltage() const;

    /**
     * \brief Gets the current output of the user-provided feedforward controller
     *
     * \return Millivolts
     */
    std::int32_t get_estimator_voltage() const;

    /**
     * \brief Gets the current output of the user-provided TBH controller
     *
     * \return Millivolts
     */
    std::int32_t get_tbh_voltage() const;

    /**
     * \brief Gets the current raw motor position
     *
     * \return Ticks
     */
    double get_position() const;

    /**
     * \brief Gets the current target motor position
     *
     * \return Ticks
     */
    double get_position_target() const;

    /**
     * \brief Gets the current motor velocity
     *
     * Uses the sylib velocity estimator
     *
     * \return RPM
     */
    double get_velocity() const;

    /**
     * \brief Gets the current motor velocity target
     *
     * \return RPM
     */
    double get_velocity_target() const;

    /**
     * \brief Gets the current motor velocity as reported by the motor itself
     *
     * This number is complete garbage and should not be used for anything.
     *
     * \return RPM
     */
    double get_velocity_motor_reported() const;

    /**
     * \brief Gets the unfiltered motor velocity
     *
     * Uses the sylib velocity estimator
     *
     * \return RPM
     */
    double get_velocity_raw() const;

    /**
     * \brief Gets the partially-filtered motor velocity
     *
     * Uses the sylib velocity estimator
     *
     * \return RPM
     */
    double get_velocity_sma_filter_only() const;

    /**
     * \brief Gets the motor acceleration
     *
     * Uses the sylib velocity estimator
     *
     * \return RPM per millisecond
     */
    double get_acceleration() const;

    /**
     * \brief Gets the motor temperature
     *
     * \return Degrees Celsius, rounded to the nearest 5
     */
    double get_temperature() const;

    /**
     * \brief Gets the motor torque
     *
     * \return Newton-Metres
     */
    double get_torque() const;

    /**
     * \brief Gets the motor power consumption
     *
     * \return Watts
     */
    double get_watts() const;

    /**
     * \brief Gets the motor current draw
     *
     * \return Amps
     */
    std::int32_t get_amps() const;

    /**
     * \brief Gets the user-provided motor current limit
     *
     * \return Amps
     */
    std::int32_t get_amps_limit() const;

    /**
     * \brief Gets the voltage most recently sent to the motor,
     * when using a sylib control mode
     *
     * \return Millivolts
     */
    std::int32_t get_applied_voltage() const;

    /**
     * \brief Gets the user-provided motor voltage limit
     *
     * \return Millivolts
     */
    std::int32_t get_volts_limit() const;

    /**
     * \brief Gets the motor-reported efficiency value in percent
     *
     * 100% efficiency means the motor is moving while drawing no power,
     * and 0% efficiency means the motor is drawing power while not moving.
     *
     * \return Millivolts
     */
    std::int32_t get_efficiency() const;

    /**
     * \brief Gets the motor-reported faults
     *
     * \return Who knows ¯\\_(ツ)_/¯
     */
    std::int32_t get_faults() const;

    /**
     * \brief Gets the motor-reported flags
     *
     * \return Who knows ¯\\_(ツ)_/¯
     */
    std::int32_t get_flags() const;

    /**
     * \brief Gets the most recent time at which the motor has updated its tick count
     *
     * This value is based on the motor's internal clock, and drifts from the brain's system
     * timer by approximately 1 part in 95.
     *
     * \return Milliseconds
     */
    std::uint32_t get_device_timestamp() const;

    /**
     * \brief Gets the current motor tick count and timestamp value
     *
     * Sets the value at the pointer provided to the motor timestamp in milliseconds
     *
     * \return Ticks
     */
    std::int32_t get_position_and_timestamp(std::uint32_t* timestamp);

    /**
     * \brief Gets the current motor brake mode
     *
     * \return V5MotorBrakeMode
     */
    std::int32_t get_braking_mode() const;

    /**
     * \brief Gets the user-set motor gearing
     *
     * \return RPM
     */
    std::int32_t get_gearing() const;

    /**
     * \brief Gets 1-indexed smart port used by the motor
     *
     * \return Smart Port Number
     */
    std::int32_t get_smart_port() const;

    /**
     * \brief Gets if the motor is stopped
     *
     * \return True if the most recent raw velocity measurement is 0, otherwise False
     */
    bool is_stopped() const;

    /**
     * \brief Gets if the motor is over current, as reported by the motor
     *
     * \return True or False
     */
    bool is_over_current() const;

    /**
     * \brief Gets if the motor is over temperature, as reported by the motor
     *
     * \return True or False
     */
    bool is_over_temp() const;

    /**
     * \brief Gets if the motor reversed flag is set
     *
     * \return True or False
     */
    bool is_reversed() const;
};
}  // namespace sylib
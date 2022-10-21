/**
 * \file src/sylib/math.cpp
 *
 * \brief Contains definitions for functions relating to smart motor
 * control and telemetry
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


#include "sylib/motor.hpp"
#include "sylib/env.hpp"
#include "system.hpp"


namespace sylib {
SylviesPogVelocityEstimator::SylviesPogVelocityEstimator(double motorGearing)
    : smaFilterVelocity(3),
      smaFilterAccel(15),
      smaFilterJerk(3),
      smaFilterSnap(3),
      medianFilter(7, 2, 1),
      emaFilter(),
      motorGearing(motorGearing),
      preFilterAccelSolver(),
      preFilterJerkSolver(),
      preFilterSnapSolver(),
      outputAccelSolver(),
      outputJerkSolver(),
      outputSnapSolver(),
      maxFilterAccel(20) {
    previousInternalMotorClock = 0;
    oldMotorTicks = 0;
    dN = 0;
    dT = 0;
    rawVelocity = 0;
    motorVoltageTarget = 0;
    emaFilteredVelocity = 0;
    smaFilteredVelocity = 0;
    smaFilteredAccel = 0;
    medianFilteredVelocity = 0;
    outputVelocity = 0;
    preFilterAcceleration = 0;
    preFilterJerk = 0;
    preFilterSnap = 0;
    outputAcceleration = 0;
    outputJerk = 0;
    outputSnap = 0;
    accelCalculatedkA = 0;
    maxFilteredAccel = 0;
}

double SylviesPogVelocityEstimator::getVelocity(double currentMotorTicks,
                                                std::uint32_t currentInternalMotorClock) {
    dT = 5 * std::round((currentInternalMotorClock - previousInternalMotorClock) / 5.0);
    if (dT == 0) {
        return outputVelocity;
    }
    static uint32_t prevTime = sylib::millis();
    dN = currentMotorTicks - oldMotorTicks;
    previousInternalMotorClock = currentInternalMotorClock;
    oldMotorTicks = currentMotorTicks;
    rawVelocity = (dN / 50) / dT * 60000;
    if (std::abs(rawVelocity) > 5000) {  // Motor position reset manually
        // return outputVelocity;
    }
    smaFilteredVelocity = smaFilterVelocity.filter(rawVelocity);
    medianFilteredVelocity = medianFilter.filter(smaFilteredVelocity);

    preFilterAcceleration = preFilterAccelSolver.solveDerivative(medianFilteredVelocity);
    preFilterJerk = preFilterJerkSolver.solveDerivative(preFilterAcceleration);
    preFilterSnap = preFilterSnapSolver.solveDerivative(preFilterJerk);

    maxFilteredAccel = maxFilterAccel.filter(preFilterAcceleration);
    smaFilteredAccel = smaFilterAccel.filter(preFilterAcceleration);
    smaFilteredJerk = smaFilterJerk.filter(preFilterJerk);
    smaFilteredSnap = smaFilterSnap.filter(preFilterSnap);

    accelCalculatedkA = 0.75 * (1 - (1 / ((maxFilteredAccel * maxFilteredAccel / 50) + 1.013)));

    emaFilteredVelocity = emaFilter.filter(smaFilteredVelocity, accelCalculatedkA);
    outputVelocity = emaFilteredVelocity * motorGearing / 3600;
    outputAcceleration = outputAccelSolver.solveDerivative(outputVelocity);
    outputJerk = outputJerkSolver.solveDerivative(outputAcceleration);
    outputSnap = outputSnapSolver.solveDerivative(outputJerk);

    if(sylib::millis()-prevTime != 10){
        // printf("%d|%d|%f\n", sylib::millis(), sylib::millis()-prevTime, dT);
    }
    prevTime = sylib::millis();
    return outputVelocity;
}

double SylviesPogVelocityEstimator::getVelocityNoCalculations() const { return outputVelocity; }

double SylviesPogVelocityEstimator::getRawVelocity() const { return rawVelocity; }

double SylviesPogVelocityEstimator::getRawPosition() const { return oldMotorTicks; }

double SylviesPogVelocityEstimator::getEmaFilteredVelocity() const { return emaFilteredVelocity; }

double SylviesPogVelocityEstimator::getSmaFilteredVelocity() const { return smaFilteredVelocity; }

double SylviesPogVelocityEstimator::getMedianFilteredVelocity() const {
    return medianFilteredVelocity;
}

double SylviesPogVelocityEstimator::getAcceleration() const { return outputAcceleration; }

double SylviesPogVelocityEstimator::getPreFilterAcceleration() const {
    return preFilterAcceleration;
}

double SylviesPogVelocityEstimator::getJerk() const { return outputJerk; }

double SylviesPogVelocityEstimator::getSnap() const { return outputSnap; }

double SylviesPogVelocityEstimator::getCalculatedkA() const { return accelCalculatedkA; }

double SylviesPogVelocityEstimator::getMaxFilteredAcceleration() const { return maxFilteredAccel; }

Motor::Motor(const uint8_t smart_port, const double gearing, const bool reverse,
             const SpeedControllerInfo speedController)
    : Device(5, 0),
      smart_port(smart_port),
      gearing(gearing),
      reversed(reverse),
      device(vexDeviceGetByIndex(smart_port - 1)),
      motorVoltageEstimator{speedController.kV, gearing},
      motorPController{speedController.kP,
                       std::shared_ptr<double>(&velocity_error),
                       gearing,
                       true,
                       speedController.kP2,
                       speedController.pRange},
      motorIController{speedController.kI, std::shared_ptr<double>(&velocity_error), gearing, true,
                       speedController.antiWindupRange},
      motorDController{speedController.kD, std::shared_ptr<double>(&velocity_error), gearing},
      motorTBHController{speedController.kH, std::shared_ptr<double>(&velocity_error)},
      speedController(speedController) {
    std::unique_lock<sylib::Mutex> _lock;
    if (sylib::millis() > 1) {
        _lock = std::unique_lock<sylib::Mutex>(sylib_port_mutexes[smart_port - 1]);
    }
    motorVelocityTracker = sylib::SylviesPogVelocityEstimator(gearing);
    vexDeviceMotorGearingSet(device, kMotorGearSet_18);
}

void Motor::set_motor_controller() {
    static SylibMotorControlMode previousMode;
    if (sylib_control_mode != previousMode) {
        motorIController.resetValue();
    }
    previousMode = sylib_control_mode;
    if (sylib_control_mode == SylibMotorControlModeOFF) {
        vexDeviceMotorBrakeModeSet(device, kV5MotorBrakeModeCoast);
        vexDeviceMotorVelocitySet(device, 0);
    } else if (sylib_control_mode == SylibMotorControlModeBRAKE) {
        vexDeviceMotorBrakeModeSet(device, kV5MotorBrakeModeBrake);
        vexDeviceMotorVelocitySet(device, 0);
    } else if (sylib_control_mode == SylibMotorControlModeHOLD) {
        vexDeviceMotorBrakeModeSet(device, kV5MotorBrakeModeHold);
        vexDeviceMotorVelocitySet(device, 0);
    } else if (sylib_control_mode == SylibMotorControlModeCUSTOM) {
        if (((velocity_target > 0)) &&
            (velocity_error > speedController.maxVoltageRange * gearing / 3600)) {
            voltage_target = 12000;
        } else if (((velocity_target < 0)) &&
                   (velocity_error < speedController.maxVoltageRange * gearing / 3600)) {
            voltage_target = -12000;
        } else {
            voltage_target = motorVoltageEstimator.estimate(velocity_target) + *motorPController +
                             *motorIController + *motorDController + *motorTBHController;
        }

        if (speedController.coastDownEnabled) {
            if ((velocity_target > 0) && velocity_error < -speedController.coastDownRange) {
                voltage_target *= speedController.coastDownModifier;
            } else if ((velocity_target < 0) && velocity_target > speedController.coastDownRange) {
                voltage_target *= speedController.coastDownModifier;
            }
        }

        if (velocity_target * voltage_target <= 0) {
            voltage_target = 0;
        }

        if (voltage_target > 0 && voltage_target > 12000) {
            voltage_target = 12000;
        } else if (voltage_target < 0 && voltage_target < -12000) {
            voltage_target = -12000;
        }
        if (!reversed) {
            vexDeviceMotorVoltageSet(device, voltage_target);
        } else {
            vexDeviceMotorVoltageSet(device, -voltage_target);
        }

    } else if (sylib_control_mode == SylibMotorControlModePOSITION) {
        // Unused
    } else if (sylib_control_mode == SylibMotorControlModeVelocityAUTO) {
        vexDeviceMotorVelocitySet(device, velocity_target);
    } else if (sylib_control_mode == SylibMotorControlModePositionAUTO) {
        vexDeviceMotorPositionSet(device, position_target);
    } else if (sylib_control_mode == SylibMotorControlModeVOLTAGE) {
        vexDeviceMotorVoltageSet(device, voltage_target);
    }
}

void Motor::update() {
    // static int prevUpdateTime;
    // static int updateTime;
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);

    // updateTime = sylib::millis();
    // printf("%d\n", updateTime-prevUpdateTime);
    // prevUpdateTime = updateTime;

    position = vexDeviceMotorPositionRawGet(device, &internalClock);

    if (!reversed) {
        velocity = motorVelocityTracker.getVelocity(position, internalClock);
    } else {
        velocity = -motorVelocityTracker.getVelocity(position, internalClock);
    }
    if(sylib_control_mode == SylibMotorControlModeVelocityAUTO){
        velocity_error = velocity_target*gearing/200 - velocity;
    }
    else{
        velocity_error = velocity_target - velocity;
    }
    motorPController.update();
    motorIController.update();
    motorDController.update();
    motorTBHController.update();
    raw_velocity = motorVelocityTracker.getRawVelocity();
    vexos_velocity = vexDeviceMotorActualVelocityGet(device);
    sma_velocity = motorVelocityTracker.getSmaFilteredVelocity();
    acceleration = motorVelocityTracker.getAcceleration();
    temperature = vexDeviceMotorTemperatureGet(device);
    power = vexDeviceMotorPowerGet(device);
    current_draw = vexDeviceMotorCurrentGet(device);
    direction = vexDeviceMotorDirectionGet(device);
    efficiency = vexDeviceMotorEfficiencyGet(device);
    faults = vexDeviceMotorFaultsGet(device);
    flags = vexDeviceMotorFlagsGet(device);
    torque = vexDeviceMotorTorqueGet(device);
    voltage = vexDeviceMotorVoltageGet(device);
    zero_position_flag = vexDeviceMotorZeroPositionFlagGet(device);
    over_temp = vexDeviceMotorOverTempFlagGet(device);
    over_current = vexDeviceMotorCurrentLimitFlagGet(device);
    brake_mode = vexDeviceMotorBrakeModeGet(device);
    current_limit = vexDeviceMotorCurrentLimitGet(device);
    voltage_limit = vexDeviceMotorVoltageLimitGet(device);
    set_motor_controller();
}

void Motor::set_position_target_absolute(double new_position, std::int32_t velocity_cap) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    position_target = new_position;
    velocity_target = velocity_cap * 200 / gearing;
    sylib_control_mode = SylibMotorControlModePositionAUTO;
}

void Motor::set_position_target_relative(double new_position, std::int32_t velocity_cap) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    if (new_position != position_target) {
        position_target = position + new_position;
    }
    velocity_target = velocity_cap * 200 / gearing;
    sylib_control_mode = SylibMotorControlModePositionAUTO;
}

void Motor::set_velocity_custom_controller(std::int16_t new_velocity_target) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);

    velocity_target = new_velocity_target;

    sylib_control_mode = SylibMotorControlModeCUSTOM;
}

void Motor::stop() {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    if (brake_mode == kV5MotorBrakeModeCoast)
        sylib_control_mode = SylibMotorControlModeOFF;
    else if (brake_mode == kV5MotorBrakeModeBrake)
        sylib_control_mode = SylibMotorControlModeBRAKE;
    else if (brake_mode == kV5MotorBrakeModeHold)
        sylib_control_mode = SylibMotorControlModeHOLD;
    else
        sylib_control_mode = SylibMotorControlModeOFF;
}

void Motor::set_voltage(std::int16_t new_voltage_target) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    voltage_target = new_voltage_target;
    sylib_control_mode = SylibMotorControlModeVOLTAGE;
}

void Motor::set_braking_mode(V5MotorBrakeMode mode) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    brake_mode = mode;
    vexDeviceMotorBrakeModeSet(device, mode);
}

void Motor::set_amps_limit(std::int32_t limit) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    vexDeviceMotorCurrentLimitSet(device, limit);
}

void Motor::set_is_reversed(bool reverse) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    reversed = reverse;
    vexDeviceMotorReverseFlagSet(device, reversed);
}

void Motor::set_volts_limit(std::int32_t limit) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    vexDeviceMotorVoltageLimitSet(device, limit);
}

void Motor::tare_encoder() {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    vexDeviceMotorPositionReset(device);
    position = 0;
}

void Motor::set_velocity(std::int32_t new_velocity_target) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    velocity_target = new_velocity_target * 200 / gearing;
    sylib_control_mode = SylibMotorControlModeVelocityAUTO;
}

void Motor::updateSpeedController(sylib::SpeedControllerInfo controller) {
    motorVoltageEstimator.setkV(controller.kV);
    motorPController.setkP(controller.kP);
    motorIController.setkI(controller.kI);
    motorDController.setkD(controller.kD);
    motorPController.setkP2(controller.kP2);
    motorPController.setMaxRangeEnabled(controller.pRange);
    motorPController.setMaxRange(controller.pRange);
    motorIController.setAntiWindupEnabled(controller.antiWindupEnabled);
    motorIController.setAntiWindupRange(controller.antiWindupRange);
    speedController = controller;
}

double Motor::get_velocity_error() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return velocity_error;
}

std::int32_t Motor::get_p_voltage() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return motorPController.getOutput();
}

std::int32_t Motor::get_i_voltage() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return motorIController.getOutput();
}

std::int32_t Motor::get_d_voltage() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return motorDController.getOutput();
}

std::int32_t Motor::get_estimator_voltage() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return motorVoltageEstimator.getOutput();
}

std::int32_t Motor::get_tbh_voltage() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return motorTBHController.getOutput();
}

double Motor::get_velocity() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return velocity;
}

double Motor::get_velocity_motor_reported() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return vexos_velocity * gearing / 200;
}

double Motor::get_velocity_sma_filter_only() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return sma_velocity;
}

double Motor::get_velocity_target() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return velocity_target;
}

double Motor::get_velocity_raw() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return raw_velocity;
}

double Motor::get_acceleration() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return acceleration;
}

double Motor::get_position() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return position;
}

double Motor::get_position_target() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return position_target;
}

double Motor::get_temperature() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return temperature;
}

double Motor::get_torque() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return torque;
}

double Motor::get_watts() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return power;
}

std::int32_t Motor::get_amps() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return current_draw;
}

std::int32_t Motor::get_amps_limit() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return current_limit;
}

std::int32_t Motor::get_applied_voltage() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return voltage_target;
}

std::int32_t Motor::get_volts_limit() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return voltage_limit;
}

std::int32_t Motor::get_efficiency() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return efficiency;
}

std::int32_t Motor::get_faults() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return faults;
}

std::int32_t Motor::get_flags() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return flags;
}

std::uint32_t Motor::get_device_timestamp() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return internalClock;
}

std::int32_t Motor::get_braking_mode() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return brake_mode;
}

std::int32_t Motor::get_gearing() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return gearing;
}

std::int32_t Motor::get_smart_port() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return smart_port;
}

bool Motor::is_stopped() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return (raw_velocity == 0);
}

bool Motor::is_over_current() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return over_current;
}

bool Motor::is_over_temp() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return over_temp;
}

bool Motor::is_reversed() const {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    return reversed;
}

std::int32_t Motor::get_position_and_timestamp(std::uint32_t* timestamp) {
    mutex_lock _lock(sylib_port_mutexes[smart_port - 1]);
    *timestamp = internalClock;
    return position;
}
}  // namespace sylib
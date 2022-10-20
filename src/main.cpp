#include "main.h"
#include "sylib/motor.hpp"
#include "sylib/system.hpp"
#include <cstdint>
#include <vector>


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void initialize() {
	// Initialize sylib background processes
	sylib::initialize();
}

sylib::SpeedControllerInfo flywheelController (
        [](double rpm){return std::pow(M_E, (-0.001*rpm* 3600 / 3600 + 1)) + 3.125;}, // kV function
        10, // kP
        0.001, // kI
        0, // kD
        0, // kH
        true, // anti-windup enabled
        50, // anti-windup range
        true, // p controller bounds threshold enabled
        50, // p controller bounds cutoff enabled
        0.01, // kP2 for when over threshold
        50, // range to target to apply max voltage
        false, // coast down enabled
        0,  // coast down theshhold
        1 // coast down constant
);

void opcontrol() {
    
	// Create an addrled object

	// auto flywheel = sylib::Motor(17, 3600, true, flywheelController);
	
	// Store the time at the start of the loop
    std::uint32_t clock = sylib::millis();
    while (true) {
		// flywheel.set_velocity_custom_controller(2700);
        // flywheel.set_velocity_custom_controller(0);
		// 10ms delay to allow other tasks to run

        sylib::delay_until(&clock, 10);
    }
}

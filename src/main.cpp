#include "main.h"
#include "sylib/system.hpp"
#include <cstdint>
#include <vector>


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	sylib::initialize();
}

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
auto led1 = sylib::Addrled(1, 1, 24);

void opcontrol() {
    led1.gradient(0x00FF00, 0x0000FF, 0, 0, false, true);
    led1.cycle(*led1, 15);
	auto colors = std::vector<std::uint32_t>();
	colors.resize(16);


	for(int i = 0; i < colors.size(); i++){
		colors[i] = sylib::Addrled::rgb_to_hex(64, 64, i * 15);
	}
	led1.set_buffer(colors);

	led1.color_shift(64 , 0, -127);
	
    std::uint32_t current_time = sylib::millis();
    while (true) {
        sylib::delay_until(&current_time, 10);
    }
}

void initialize() 
{
	sylib::SylibDaemon::startSylibDaemon();
}


void opcontrol() {
	
	
	// A collection of settings for a speed controller
	//
	// These values need to be tuned by each user, the sample constants
	// provided will not be useful.
	sylib::SpeedControllerInfo motor_speed_controller (
        [](double rpm){return 5;}, // kV function
        1, // kP
        1, // kI
        1, // kD
        1, // kH
        false, // anti-windup enabled
        0, // anti-windup range
        false, // p controller bounds threshold enabled
        0, // p controller bounds cutoff enabled
        1, // kP2 for when over threshold
        0 // range to target to apply max voltage
	);

	// Create a motor object on port 17, with a green cart, set as reversed, and with
	// motor_speed_controller as a custom velocity controller
	auto motor = sylib::Motor(17,200, true, motor_speed_controller);

	sylib::delay(1000); // 1 second delay

	// Set the motor to spin at 150 RPM for 5 second using the motor's 
	// built-in speed controller. This is analogous to move_velocity in PROS,
	// and setVelocity in VEXcode 
	motor.set_velocity(150); 
	sylib::delay(5000);
	motor.stop();

	// Set the motor to spin at 150 RPM for 5 second using the 
	// speed controller built by the user and supplied in the construction
	// of the motor object.
	motor.set_velocity_custom_controller(150); 
	sylib::delay(5000);
	motor.stop();

	// Set the motor to maximum voltage for 5 seconds
	motor.set_voltage(12000); 
	sylib::delay(5000);
	motor.stop();
	
	// Store the time at the start of the loop
	uint32_t clock = sylib::millis();
	while (true){
		sylib::delay_until(&clock,10);
	}
}
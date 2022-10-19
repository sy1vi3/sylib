# Sample Code

Some example projects using sylib for various basic tasks.

## Controlling a Smart Motor

```cpp
void initialize() {
	// Initialize sylib background processes
	sylib::initialize();
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
```

## Outputting Motor Telemetry to the PROS Terminal

```cpp
void initialize() {
	// Initialize sylib background processes
	sylib::initialize();
}

void opcontrol() {
    
	// Create a motor object
	auto motor = sylib::Motor(4,600);

	// Use the motor's internal velocity controller to set a speed target
	motor.set_velocity(550);
	
	// Store the time at the start of the loop
    std::uint32_t clock = sylib::millis();
    while (true) {

		// Print values in CSV format to the terminal
		// Requires a tethered connnection to the brain
		printf("%d,%f,%f,%f,%d\n", sylib::millis(), motor.get_velocity(), motor.get_velocity_error(), motor.get_acceleration(), motor.get_efficiency());

		// 10ms delay to allow other tasks to run
        sylib::delay_until(&clock, 10);
    }
}
```

This output can be piped into a CSV file using the method of your choosing.
With a small amount of python code, it is trivial to generate graphs from this output

![image](https://user-images.githubusercontent.com/54775775/196566142-75490337-9dd6-4088-946a-47b65936e8c3.png)


## Creating a Rainbow Pattern on LED Lights

```cpp
void initialize() {
	// Initialize sylib background processes
	sylib::initialize();
}

void opcontrol() {
    
	// Create an addrled object
	auto addrled = sylib::Addrled(1,1,64);

	// Set the LED strip to a gradient in HSV color space
	// that displays a full range of hues
	addrled.gradient(0xFF0000, 0xFF0005, 0, 0, false, true);

	// Cycle the colors at speed 10
	addrled.cycle(*addrled, 10);
	
	// Store the time at the start of the loop
    std::uint32_t clock = sylib::millis();
    while (true) {
		
		// 10ms delay to allow other tasks to run
        sylib::delay_until(&clock, 10);
    }
}
```




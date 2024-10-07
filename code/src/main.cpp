#include "main.h"
#include <stdlib.h>
#include <iostream>

void move_forward(int distance, int speed) {
    left_mg.move_relative(distance, speed);
    right_mg.move_relative(distance, speed);
    pros::delay(2000);  // Wait for the movement to complete
}

void turn_robot(int angle) {
    int turn_distance = angle * 10; // Adjust the multiplier based on how much turning is needed
    left_mg.move(turn_distance);
    right_mg.move(-turn_distance);
    pros::delay(1000);
}

// Function to grab the stake
void grab_stake() {
    grabber_motor.move_velocity(100);  // Close the grabber motor to grab the stake
    pros::delay(GRAB_TIME);            // Wait for grabber to close
    grabber_motor.move_velocity(0);    // Stop the grabber motor
}

// Function to release the stake
void release_stake() {
    grabber_motor.move_velocity(-100);  // Open the grabber to release the stake
    pros::delay(GRAB_TIME);             // Wait for grabber to open
    grabber_motor.move_velocity(0);     // Stop the grabber motor
}


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	autonomous();
	// pros::ADIAnalogIn sensor (ANALOG_SENSOR_PORT);
  	// sensor.calibrate();

	// pros::Motor drive_left_initializer (MOTOR_PORT, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
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
void autonomous() {
	int MOTOR_MAX_SPEED = 100;

	pros::lcd::set_text(5, "Entering autonomous");

	pros::MotorGroup left_mg({1, 2, -8});   
	pros::MotorGroup right_mg({-4, -5, 7});

	right_mg.move(50000);
	left_mg.move(50000);
	pros::delay(2000);

	right_mg.move(5000);
	pros::delay(2000);
	right_mg.move(5000);
	left_mg.move(5000);

}

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
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, 2, -8});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, -5, 7});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		pros::lcd::set_text(3, "Turn: " + std::to_string(turn) + " Dir: " + std::to_string(dir));
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}
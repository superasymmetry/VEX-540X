#include "main.h"
#include <stdlib.h>
#include <iostream>
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"

// void autonomous3();

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({8, 18, -13}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-9, -14, 16}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intake(20, pros::MotorGearset::green);
pros::Motor conveyor(2, pros::MotorGearset::green);
bool grab = false;
bool ext = false;
bool intaking = false;
pros::ADIDigitalOut grabber_motor('G', grab);
pros::ADIDigitalOut stick('H', ext);
pros::IMU imu(19);


lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              10,
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void move_forward(int distance, int speed) {
	distance *= 5;
    left_mg.move_relative(distance, speed);
    right_mg.move_relative(distance, speed);
    pros::delay((distance / abs(speed)) * 1000);
}

void turn_robot(int angle) {
    int turn_distance = angle * 10;
    left_mg.move(turn_distance);
    right_mg.move(-turn_distance);
    pros::delay(1000);
}

void grab_stake() {
	grab = !grab;
	grabber_motor.set_value(grab);
	pros::delay(200);
}

void stick_change() {
	ext = !ext;
	stick.set_value(ext);
	pros::delay(200);
}

void score(){
	intake.move(127);
	pros::delay(2000);
	intake.move(0);
}

void toggle_intake(){
	intaking = !intaking;
	intake.move(200*intaking);
	conveyor.move(500*intaking);
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
	chassis.calibrate();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });

	// imu.reset();
	// pros::delay(3000);
	// imu.set_heading(0);

	// autonomous();
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
	int MOTOR_MAX_SPEED = 127;

	pros::lcd::set_text(5, "Entering autonomous");

    // chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(90, 100000);

	// move_forward(10,127);
	pros::lcd::set_text(6,"move");
	grab_stake();
	// pros::delay(100);
	// turn_robot(-4);
	// move_forward(80,127);
	// grab_stake();
	// turn_robot(6);
	// move_forward(150,127);

	// chassis.setPose(47, -65, 0);
	// pros::lcd::set_text(6, "set pose");
	// pros::delay(450);

    // chassis.turnToHeading(-90, 1000);
	// pros::lcd::set_text(6, "turn to heading");
	// pros::delay(450);


	// chassis.moveToPoint(13, -65, 100000);
	// pros::lcd::set_text(6, "move1");
	// pros::delay(450); 

	// grab_stake();
	// chassis.moveToPose(19,-47.5, 270, 1000);
	// pros::lcd::set_text(6, "move2");
	// pros::delay(450);
	// chassis.moveToPose(23, -30, 180, 1000);
	// pros::lcd::set_text(6, "move3");
	// pros::delay(450);
	// chassis.moveToPoint(17, -35, 1000);
	// pros::lcd::set_text(6, "move4");
}

void autonomous2(){
	pros::lcd::set_text(6, "2nd autonomous");
	chassis.setPose(65,23,90);
	pros::delay(450);

	chassis.moveToPose(32,23,270,10000);
	pros::delay(450);
	grab_stake();

	chassis.moveToPose(8,40,125,10000);
	chassis.moveToPose(8,50,125,10000);

}

void autonomous3(){
	move_forward(10,127);
	pros::lcd::set_text(6,"move");
	grab_stake();
	pros::delay(100);
	turn_robot(-4);
	move_forward(80,127);
	grab_stake();
	turn_robot(6);
	move_forward(150,127);
	grab_stake();
	pros::delay(100);
	turn_robot(1);
	move_forward(30,127);
}

void autonomous4() {  // autonomous skills 3 from teams chat
	pros::lcd::set_text(6, "skills auton 2");
	chassis.setPose(-59, 0, 270);
	pros::delay(500);

	chassis.moveToPose(-47, 0, 270, 10000);
	chassis.turnToHeading(0, 10000);
	pros::delay(250);
	chassis.moveToPose(-47, -17, 0, 10000);  // moving into position to grab stake
	grab_stake();

	chassis.moveToPose(-47, -23.5, 90, 10000);  // moving into stake location to align with first ring
	pros::delay(250);
	toggle_intake();
	chassis.moveToPose(-23.5, -23.5, 90, 10000);  // first ring location
	chassis.turnToHeading(180, 10000);
	pros::delay(250);
	chassis.moveToPose(-23.5, -47, 180, 10000);  // second ring location
	chassis.turnToHeading(270, 10000);
	pros::delay(250);
	chassis.moveToPose(-47, -47, 270, 10000);  // third ring location
	chassis.moveToPose(-59, -47, 270, 10000);  // fourth ring location
	pros::delay(250);
	chassis.moveToPose(-47, -47, 270, 10000);  // moving back to align with fifth ring
	chassis.turnToHeading(180, 10000);
	pros::delay(250);
	chassis.moveToPose(-47, -59, 180, 10000);  // fifth ring location
	chassis.turnToHeading(252, 10000);
	pros::delay(250);
	chassis.moveToPose(-65, -65, 252, 10000);  // moving into bottom left corner
	toggle_intake();
	grab_stake();  // letting go of stake
	
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

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); 

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			left_mg.move(500);
			right_mg.move(500);
			pros::delay(20);
			left_mg.move(0);
			right_mg.move(0);
		}else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			left_mg.move(-500);
			right_mg.move(-500);
			pros::delay(20);
			left_mg.move(0);
			right_mg.move(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(200);
			pros::delay(20);
			conveyor.move(500);
			pros::delay(20);
		}else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(-200);
			pros::delay(20);
			conveyor.move(-500);
			pros::delay(20);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			intake.move(0);
			conveyor.move(0);
		}

		int dir = -(master.get_analog(ANALOG_LEFT_Y));    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		pros::lcd::set_text(3, "Turn: " + std::to_string(turn) + " Dir: " + std::to_string(dir));
		left_mg.move((dir) - turn);                      // Sets left motor voltage
		right_mg.move((dir) + turn);                     // Sets right motor voltage
		pros::delay(20);
		// intake.move_velocity(100);
		// pros::delay(20);                               // Run for 20 ms then update

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			grab_stake();
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			stick_change();
		}
	}	

}
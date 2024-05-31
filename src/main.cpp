#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

// Project Initialized on 5/26/2024, Using 3.8.3 Pros Kernel, by Arnav Sheth (USE CLANGD NOT INTELLISENSE)

 /**
	* Motor Declarations
 */
pros::Motor lf(-2, pros::E_MOTOR_GEAR_BLUE); // Reversed
pros::Motor lm(-3, pros::E_MOTOR_GEAR_BLUE); // Reversed
pros::Motor lb(-7, pros::E_MOTOR_GEAR_BLUE); // Reversed
pros::Motor rf(1, pros::E_MOTOR_GEAR_BLUE);
pros::Motor rm(11, pros::E_MOTOR_GEAR_BLUE);
pros::Motor rb(6, pros::E_MOTOR_GEAR_BLUE);

pros::MotorGroup left_motor_group({ lf, lm, lb });
pros::MotorGroup right_motor_group({ rf, rm, rb });


/**
	* Drivetrain Settings
*/
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
							  &right_motor_group, // right motor group
							  11.25, // 11.25 inch track width
							  lemlib::Omniwheel::NEW_275, // 2.75 inch wheel diameter
							  450, // 450 rpm drive speed
							  2 // horizontal drive (CHANGE LATER)
							  );

/**
	* IMU and Tracking Declarations

	* Can use either rotation sensor or paired encoder ports ('A', 'B') - Check LemLib Documentation fore more info
*/
pros::Imu imu(5); // Port 5
pros::Rotation vertical_sensor(15); // Port 15 and Reversed
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, -0.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
							nullptr, // vertical tracking wheel 2, that does not exist - set to null
							nullptr, // horizontal tracking wheel 1, that does not exist - set to null
							nullptr, // horizontal tracking wheel 2, that does not exist - set to null
							&imu // inertial sensor
);
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
	* PID Constants
*/

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
lemlib::ControllerSettings angular_controller(4.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

/**
	* Chassis Constructor
*/

lemlib::Chassis chassis(drivetrain, // drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors // odometry settings
					    );

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
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
	// set pos to x: 0, y: 0, theta: 0
	chassis.setPose(0,0,0);
	// turn to face 90 with very long timeout
	chassis.turnToHeading(90, 100000);

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


pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX, false, 0.5);

        // delay to save resources
        pros::delay(25);
    }
}

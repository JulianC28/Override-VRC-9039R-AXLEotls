#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor rightDrive1(-1);
pros::Motor rightDrive2(-2);
pros::Motor leftDrive1(3);
pros::Motor leftDrive2(4);

pros::MotorGroup rightDrive({1, -2}, pros::MotorGearset::blue);
pros::MotorGroup leftDrive({3, 4}, pros::MotorGearset::blue);

pros::Motor leftLift(5);
pros::Motor rightLift(-6);

pros::MotorGroup lift({5, -6}, pros::MotorGearset::red);


pros::Motor intake(7);



pros::AIVision AISensor(10);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
							  &rightDrive, // right motor group
							  15, // 15 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  400, // drivetrain rpm is 480 rpm
							  2 // horizontal drift is 2
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
  							nullptr, // vertical tracking wheel 2, set to null
  							nullptr, // horizontal tracking wheel 1, set to null
  							nullptr, // horizontal tracking wheel 2, set to null
  							nullptr // inertial sensor, set to null
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                              1, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              1.275, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors // odometry sensors
);


int liftLevelCounter = 0;
void liftOneButtonFunction() {
    if(controller.get_digital(DIGITAL_L2)){
        liftLevelCounter++;
    }
    else if(controller.get_digital(DIGITAL_R2)){
        liftLevelCounter--;
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate(); // calibrate sensors
	pros::lcd::initialize();
    pros::Task::create(liftOneButtonFunction, "liftOneButton");
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


void opcontrol() {

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    lift.tare_position();

	while (true) {
		double rightDrivePower1 = rightDrive1.get_power(); // outputs the power from the first motor in the right motor group in watts
        double rightDrivePower2 = rightDrive2.get_power(); // outputs the power from the second motor in the right motor group in watts
    
        double leftDrivePower1 = leftDrive1.get_power(); // outputs the power from the first motor in the left motor group in watts
        double leftDrivePower2 = leftDrive2.get_power(); // outputs the power from the second motor in the left motor group in watts

        double leftLiftPower = leftLift.get_power();
        double rightLiftPower = rightLift.get_power();

		int batteryPercent = pros::battery::get_capacity();

        int red, green;

        if(batteryPercent > 50){
            green = 255;
            red = 510 - (batteryPercent*5.1);
        }
        else if(batteryPercent < 50){
            green = batteryPercent*5.1;
            red = 255;
        }
        else{
            green = 255;
            red = 255;
        }

        rightDrivePower1 = rightDrivePower1*1000;
        rightDrivePower2 = rightDrivePower2*1000;
        leftDrivePower1 = leftDrivePower1*1000;
        leftDrivePower2 = leftDrivePower2*1000;
        leftLiftPower = leftLiftPower*1000;
        rightLiftPower = rightLiftPower*1000;

        int rightDrivePower1Conversion = rightDrivePower1;
        int rightDrivePower2Conversion = rightDrivePower2;
        int leftDrivePower1Conversion = leftDrivePower1;
        int leftDrivePower2Conversion = leftDrivePower2;
        int leftLiftPowerConversion = leftLiftPower;
        int rightLiftPowerConversion = rightLiftPower;
        
        pros::screen::erase();
        
        pros::screen::set_pen(RGB2COLOR(red, green, 20));
        pros::screen::print(TEXT_MEDIUM, 1, "Data n' Stuff:");
        pros::screen::print(TEXT_MEDIUM, 2, "Right Drive Motor 1 (Port 1) Power: %dmW", rightDrivePower1Conversion);
        pros::screen::print(TEXT_MEDIUM, 3, "Right Drive Motor 2 (Port 2) Power: %dmW", rightDrivePower2Conversion);
        pros::screen::print(TEXT_MEDIUM, 5, "Left Drive Motor 1 (Port 3) Power: %dmW", leftDrivePower1Conversion);
        pros::screen::print(TEXT_MEDIUM, 6, "Left Drive Motor 2 (Port 4) Power: %dmW", leftDrivePower2Conversion);
        pros::screen::print(TEXT_MEDIUM, 7, "Left Lift Motor (Port 5) Power: %dmW", leftLiftPowerConversion);
        pros::screen::print(TEXT_MEDIUM, 8, "Right Lift Motor(Port 6) Power: %dmW", rightLiftPowerConversion);
        pros::screen::print(TEXT_MEDIUM, 9, "Battery Percentage: %d%", batteryPercent);
        pros::screen::print(TEXT_MEDIUM, 10, "Lift Counter Mem: %d", liftLevelCounter);

		int tankLeft = controller.get_analog(ANALOG_LEFT_Y); // the left analog stick controls the left side of the drive train
        int tankRight = controller.get_analog(ANALOG_RIGHT_Y); // the right analog stick controls the left side of the drive train        
        chassis.tank(tankLeft, tankRight); // creates a tank drive scheme

        int liftLevelCounterMemory = liftLevelCounter;

        liftOneButtonFunction();

        if(liftLevelCounterMemory != liftLevelCounter){
            pros::Task::delay(250);
        }
        
        switch(liftLevelCounter){
            case 0:
                lift.move_absolute(0, 50);
            case 1:
                lift.move_absolute(40, 50);
            case 2:
                lift.move_absolute(80, 50);
            case 3:
                lift.move_absolute(120, 50);
            case 4:
                lift.move_absolute(160, 50);
        }


        if(controller.get_digital(DIGITAL_A)){
            lift.move(40);
        }
        else if(controller.get_digital(DIGITAL_B)){
            lift.move(-40);
        }
        else{
            lift.move(0);
        }

        if(controller.get_digital(DIGITAL_L1)){
            intake.move(127);
        }
        else if(controller.get_digital(DIGITAL_R1)){
            intake.move(-127);
        }
        else{
            intake.move(0);
        }
		
		pros::delay(20);	// 20 ms delay to save resources
	}
}
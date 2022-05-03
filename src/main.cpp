#include "main.h"
#include "PID.h"
using namespace pros;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

constexpr double TURN_P = 10;
constexpr double TURN_I = 1;
constexpr double TURN_D = 0.8;
constexpr double TURN_I_MAX = 50;
//driving
constexpr double DRIVE_P = 5;
constexpr double DRIVE_I = 0; //0.0007
constexpr double DRIVE_D = 0.01; //0.005
constexpr double DRIVE_I_MAX = 30;
//secondary driving making sure angle doesn't change
constexpr double DRIVE_MATCH_P = 2;
constexpr double DRIVE_MATCH_I = 0;
constexpr double DRIVE_MATCH_D = 0;
constexpr double DRIVE_MATCH_MAX_I = 50;
pros::Motor right0(1,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right1(2,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right2(3,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor right3(4,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);


pros::Motor left0(7,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left1(8,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left2(9,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor left3(10,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor Intake(16,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor arm1(17,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm2(13,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm3(18,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm4(12,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm5(20,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);


pros::ADIDigitalOut FrontMogo (11, 'H');
pros::ADIDigitalOut BackMogo (11, 'E');
pros::ADIDigitalOut BackMogoLift (11, 'D');

pros::ADIDigitalOut horns ('D');
pros::Imu imu (6);
pros::ADIDigitalIn LiftSwitch ('B');
pros::ADIDigitalOut LiftSwitch1 ('C');
pros::ADIDigitalOut FrontBumper ('F');

std::vector<Motor> left_motors;
std::vector<Motor> right_motors;
using Motorgroup = std::vector<pros::Motor>;
double ticktoinch=49.2;


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
    imu.reset();
    do {
        pros::delay(10);
    }while (imu.is_calibrating());
    //Motors
    left0.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right0.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	pros::lcd::register_btn1_cb(on_center_button);
    left_motors.push_back(left0);
    left_motors.push_back(left1);
    left_motors.push_back(left2);
    left_motors.push_back(left3);

    right_motors.push_back(right0);
    right_motors.push_back(right1);
    right_motors.push_back(right2);
    right_motors.push_back(right3);




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
double last_target = 0;
double last_current = 0;

void run_pid(
        std::function<double()> current_function,
        const std::function<void(double)>& apply_function,
        double target,
        double p,
        double i,
        double d,
        double max_i,
        double threshold,
        uint32_t count_needed,
        uint32_t timeout_ms
){
    auto end = pros::millis() + timeout_ms;
    last_target = target;
    last_current = current_function();
    SpencerPID::PID pid{target, std::move(current_function), p, i, d, max_i};
    uint32_t count = 0;
    while (count < count_needed && (timeout_ms == 0 || pros::millis() < end)){
        auto out = pid.run();
        if (std::abs(out.error) < threshold){
            count++;
        }
        else{
            count = 0;
        }
        apply_function(out.power);
        pros::delay(5);
    }
    apply_function(0);
}
void turn_to_angle(double angle, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 0){
    run_pid(
            [](){return imu.get_heading();},
            [left_drive, right_drive, multiplier](double in_val){
                double result = std::max(std::min(in_val, 127.0), -127.0) * multiplier;
                for(auto& motor : left_drive){
                    motor = -(int32_t)result;
                }
                for(auto& motor: right_drive){
                    motor = (int32_t)result;
                }
            },
            angle,
            TURN_P, TURN_I, TURN_D, TURN_I_MAX, 3, 20, timeout_ms);
    for(auto& motor : left_drive){
        motor = 0;
    }
    for(auto& motor: right_drive){
        motor = 0;
    }
}
void drive_for_distance(int32_t distance_in, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 0){
    int32_t distance=distance_in*ticktoinch;
    auto end = pros::millis() + timeout_ms;
    const auto start_angle = imu.get_rotation();
    const auto left_start = left_drive[0].get_raw_position(nullptr);
    const auto right_start = right_drive[0].get_raw_position(nullptr);
    double real_drive_i_max = DRIVE_I_MAX/multiplier;
    SpencerPID::PID left_distance_pid{
            (double)left_start + distance,
            [&left_drive](){
                return left_drive[0].get_raw_position(nullptr);
            },
            DRIVE_P, real_drive_i_max, DRIVE_D, DRIVE_I_MAX
    };
    SpencerPID::PID right_distance_pid{
            (double)right_start + distance,
            [&right_drive](){
                return right_drive[0].get_raw_position(nullptr);
            },
            DRIVE_P, real_drive_i_max, DRIVE_D, DRIVE_I_MAX
    };
    SpencerPID::PID match_pid{
            start_angle,
            [](){
                return imu.get_rotation();
            },
            DRIVE_MATCH_P, DRIVE_MATCH_I, DRIVE_MATCH_D, DRIVE_MATCH_MAX_I
    };

    uint32_t count = 0;
    int left_stop = 0;

    constexpr double forward_backward_tolerance = 9;

    //while(count < 100 && (timeout_ms == 0 || pros::millis() < end)){
    pros::lcd::set_text(1, "Spencer is stupid");
    if(timeout_ms >0) {
        while (pros::millis() <= end) {
            pros::lcd::set_text(1, "Spencer is mad");


            auto left_result = left_distance_pid.run();
            auto right_result = right_distance_pid.run();
            auto match_result = match_pid.run();
            pros::lcd::set_text(2, std::to_string(left_result.error));
            pros::lcd::set_text(3, std::to_string(left_stop));

            if (abs(left_result.error) <= 10) {
                left_stop += 1;
            } else {
                left_stop = 0;
            }
            if (left_stop >= 100) {
                break;
            }
            if (
                    left_result.error < forward_backward_tolerance
                    && right_result.error < forward_backward_tolerance
                    && match_result.error < 3) {
                count++;
            } else {
                count = 0;
            }

            auto left = std::max(std::min(left_result.power, 127.0), -127.0) +
                        std::max(std::min(match_result.power, 127.0), -127.0);
            auto right = std::max(std::min(right_result.power, 127.0), -127.0) -
                         std::max(std::min(match_result.power, 127.0), -127.0);
            left = left * multiplier;
            right = right * multiplier;

            for (auto &motor : left_drive) {
                motor = -(int32_t) left;
            }
            for (auto &motor : right_drive) {
                motor = -(int32_t) right;
            }
            if (left_result.error == 0 and right_result.error == 0) {
                break;
            }
            pros::delay(5);
        }

        for (auto &motor : left_drive) {
            motor = 0;
        }
        for (auto &motor: right_drive) {
            motor = 0;
        }
    }
    else{
        while (true) {
            pros::lcd::set_text(1, "Spencer is mad");


            auto left_result = left_distance_pid.run();
            auto right_result = right_distance_pid.run();
            auto match_result = match_pid.run();
            pros::lcd::set_text(2, std::to_string(left_result.error));
            pros::lcd::set_text(3, std::to_string(left_stop));

            if (abs(left_result.error) <= 10) {
                left_stop += 1;
            } else {
                left_stop = 0;
            }
            if (left_stop >= 100) {
                break;
            }
            if (
                    left_result.error < forward_backward_tolerance
                    && right_result.error < forward_backward_tolerance
                    && match_result.error < 3) {
                count++;
            } else {
                count = 0;
            }

            auto left = std::max(std::min(left_result.power, 127.0), -127.0) +
                        std::max(std::min(match_result.power, 127.0), -127.0);
            auto right = std::max(std::min(right_result.power, 127.0), -127.0) -
                         std::max(std::min(match_result.power, 127.0), -127.0);
            left = left * multiplier;
            right = right * multiplier;

            for (auto &motor : left_drive) {
                motor = -(int32_t) left;
            }
            for (auto &motor : right_drive) {
                motor = -(int32_t) right;
            }
            if (left_result.error == 0 and right_result.error == 0) {
                break;
            }
            pros::delay(5);
        }

        for (auto &motor : left_drive) {
            motor = 0;
        }
        for (auto &motor: right_drive) {
            motor = 0;
        }

    }
}
void turn(double degrees, Motorgroup& left_drive, Motorgroup& right_drive, double multiplier = 1, uint32_t timeout_ms = 1000){
    const auto start = imu.get_rotation();
    const auto target = start + degrees;
    run_pid(
            [](){return imu.get_rotation();},
            [left_drive, right_drive, multiplier](double in_val){
                double result = std::max(std::min(in_val, 127.0), -127.0) * multiplier;
                for(auto& motor : left_drive){
                    motor = -(int32_t)result;
                }
                for(auto& motor: right_drive){
                    motor = (int32_t)result;
                }
            },
            target,
            TURN_P, TURN_I, TURN_D, TURN_I_MAX, 3, 20, timeout_ms);
    for(auto& motor : left_drive){
        motor = 0;
    }
    for(auto& motor: right_drive){
        motor = 0;
    }
}

void autonomous() {

    drive_for_distance(40, left_motors, right_motors, 0.75);
    turn(90, left_motors, right_motors, 1);
    drive_for_distance(40, left_motors, right_motors, 0.75);
    turn(90, left_motors, right_motors, 1);
    drive_for_distance(40, left_motors, right_motors, 0.75);
    turn(90, left_motors, right_motors, 1);
    drive_for_distance(40, left_motors, right_motors, 0.75);
    turn(90, left_motors, right_motors, 1);






}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true) {

		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

        left0 = left;
        left1 = left;
        left2 = left;
        left3 = left;

        right0 = right;
        right1 = right;
        right2 = right;
        right3 = right;

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
           while(!LiftSwitch.get_value()) {
               arm1 = -100;
               arm2 = -100;
               arm3 = -100;
               arm4 = -100;
               arm5 = -100;
           }
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                arm1 = 100;
                arm2 = 100;
                arm3 = 100;
                arm4 = 100;
                arm5 = 100;

            }
        else{
            arm1 = 0;
            arm2 = 0;
            arm3 = 0;
            arm4 = 0;
            arm5 = 0;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            Intake = 100;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            Intake = -100;

        }
        else{
            Intake = 0;
        }


        bool front_mogo = false;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            front_mogo = !front_mogo;
            FrontMogo.set_value(front_mogo);

        }
        bool rear_mogo = false;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            rear_mogo = !rear_mogo;
            BackMogo.set_value(rear_mogo);

        }
        bool back_mogo_goal_thingie = false;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            back_mogo_goal_thingie = !back_mogo_goal_thingie;
            BackMogoLift.set_value(rear_mogo);

        }



        pros::delay(20);
	}
}

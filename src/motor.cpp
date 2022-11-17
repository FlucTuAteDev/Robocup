#include "Motor.h"
#include "MotorController.h"
#include "utils.h"

void Motor::setup()
{
	pinMode(forward_pin, OUTPUT);
	pinMode(backward_pin, OUTPUT);
	pinMode(speed_pin, OUTPUT);
	pinMode(hall_pin, INPUT);
}

void Motor::set_target(int16_t speed) {
	this->target_speed = constrain(speed, -MotorController::max_motor_speed, MotorController::max_motor_speed);
}

void Motor::turn(int16_t speed) {
	this->speed = constrain(speed, -MotorController::max_motor_speed, MotorController::max_motor_speed);

	digitalWrite(forward_pin, this->speed > 0);
	digitalWrite(backward_pin, this->speed < 0);

	analogWrite(speed_pin, abs(this->speed));
}

void Motor::update_rotation_count() {
	uint8_t curr_hall_state = digitalRead(hall_pin);
	// If there is a pulse take away or add 1 to the pulse count depending on the direction
	hall_pulse_count += (prev_hall_state != curr_hall_state) * sign(speed);
	prev_hall_state = curr_hall_state;
}

void Motor::update_motor_speed() {
	int16_t diff = target_speed - speed;
	// bool dir_change = sign(target_speed) + sign(speed) == 0;

	// Yank the motor before actually moving it in the other direction
	// It compensates the bearings inaccuracy
	// if (dir_change) {
	// 	turn(sign(diff) * 255);
	// 	delay(1);
	// }
	
	int16_t change = sign(diff) * min(abs(diff), MotorController::max_change);
	// int16_t change = diff - dir_change * (diff * (speed / 255.0) + sign(diff) * MotorController::max_change);
	turn(speed + change);
}

void Motor::reset_hall() {
	hall_pulse_count = 0;
}
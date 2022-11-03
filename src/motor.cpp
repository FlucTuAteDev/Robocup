#include "motor.h"
#include "utils.h"

void Motor::setup()
{
	pinMode(forward_pin, OUTPUT);
	pinMode(backward_pin, OUTPUT);
	pinMode(speed_pin, OUTPUT);
	pinMode(hall_pin, INPUT);
}

void Motor::turn(int16_t speed)
{
	this->speed = speed;

	digitalWrite(forward_pin, speed > 0);
	digitalWrite(backward_pin, speed < 0);

	analogWrite(speed_pin, abs(speed));
}

void Motor::updateRotationCount() {
	// Check if there is a pulse
	uint8_t currHallState = digitalRead(hall_pin);
	// If there is a pulse take away or add 1 to the pulse count depending on the direction
	hall_pulse_count += (prevHallState != currHallState) * ((speed > 0) - (speed < 0));
}

void MovementController::setup()  {
	left.setup();
	right.setup();
}

void MovementController::go(int16_t speed, float diff)
{
	diff = clamp(diff, -1, 1);

	int right_speed = speed - (diff > 0) * (diff * 2 * speed);
	int left_speed = speed - (diff < 0) * (-diff * 2 * speed);
	right_speed = clamp(right_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
	left_speed = clamp(right_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

	right.turn(right_speed);
	left.turn(left_speed);
}

void MovementController::updateRotationCounts() {
	right.updateRotationCount();
	left.updateRotationCount();
}
#include "motor.h"
#include "utils.h"

int16_t Motor::MAX_CHANGE = 20;
int16_t Motor::MAX_MOTOR_SPEED = 150;

void Motor::setup()
{
	pinMode(forward_pin, OUTPUT);
	pinMode(backward_pin, OUTPUT);
	pinMode(speed_pin, OUTPUT);
	pinMode(hall_pin, INPUT);
}

void Motor::setTarget(int16_t speed) {
	this->target_speed = speed;
}

void Motor::turn(int16_t speed)
{
	this->speed = clamp<int16_t>(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

	digitalWrite(forward_pin, speed >= 0);
	digitalWrite(backward_pin, speed <= 0);

	analogWrite(speed_pin, abs(speed));
}

void Motor::updateRotationCount() {
	// Check if there is a pulse
	uint8_t currHallState = digitalRead(hall_pin);
	// If there is a pulse take away or add 1 to the pulse count depending on the direction
	hall_pulse_count += (prevHallState != currHallState) * ((speed > 0) - (speed < 0));
}

void Motor::updateMotorSpeed() {
	int16_t diff = this->target_speed - this->speed;

	int16_t change = sign(diff) * min(abs(diff), MAX_CHANGE);
	turn(speed + change);
}

void MovementController::setup()  {
	left.setup();
	right.setup();
}

void MovementController::go(int16_t speed, float diff)
{
	diff = clamp(diff, -1.0f, 1.0f);

	int16_t right_speed = speed - (diff > 0) * (diff * 2 * speed);
	int16_t left_speed = speed - (diff < 0) * (-diff * 2 * speed);

	right.turn(right_speed);
	left.turn(left_speed);
}

void MovementController::updateRotationCounts() {
	right.updateRotationCount();
	left.updateRotationCount();
}

void MovementController::updateMotorSpeeds() {
	right.updateMotorSpeed();
	left.updateMotorSpeed();
}
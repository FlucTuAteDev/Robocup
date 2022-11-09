#include "MotorController.h"
#include "utils.h"

#define MAX_MOTOR_SPEED 150
#define MAX_CHANGE 30

int16_t MotorController::max_motor_speed = MAX_MOTOR_SPEED;
float MotorController::max_change = MAX_CHANGE;
int16_t MotorController::prev_max_motor_speed = 0;

void MotorController::setup()  {
	left.setup();
	right.setup();
}

void MotorController::go(int16_t target, float diff)
{
	diff = clamp(diff, -1.0f, 1.0f);

	int16_t right_speed = target - (diff > 0) * (diff * 2 * target);
	int16_t left_speed = target - (diff < 0) * (-diff * 2 * target);

	right.set_target(right_speed);
	left.set_target(left_speed);
	// right.turn(right_speed);
	// left.turn(left_speed);
}

void MotorController::update_rotation_counts() {
	right.update_rotation_count();
	left.update_rotation_count();
}

void MotorController::update_motor_speeds() {
	right.update_motor_speed();
	left.update_motor_speed();
}

void MotorController::stop() {
	if (stopped) return;

	stopped = true;
	prev_max_motor_speed = max_motor_speed;
	max_motor_speed = 0;
}

void MotorController::start() {
	stopped = false;
	max_motor_speed = prev_max_motor_speed;
}

void MotorController::toggle() {
	stopped ? start() : stop();
}

void MotorController::reset_halls() {
	left.reset_hall();
	right.reset_hall();
}
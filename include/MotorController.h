#pragma once
#include <Motor.h>
#include <Arduino.h>

class MotorController {
public:
	Motor left, right;
	static int16_t max_motor_speed;
	static int16_t prev_max_motor_speed;
	static float max_change;
	bool stopped = false;

	MotorController()
		: left(7, 6, 9, 5), right(8, 12, 10, 4) {}

	void setup();

	void go(int16_t speed, float diff);
	void update_rotation_counts();
	void update_motor_speeds();
	void reset_halls();

	void stop();
	void start();
	void toggle();
};
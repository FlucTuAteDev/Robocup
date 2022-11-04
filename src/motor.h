#pragma once
#include <Arduino.h>
#include <math.h>

class Motor {
public:
	uint8_t forward_pin, backward_pin, speed_pin, hall_pin;
	int hall_pulse_count = 0;
	uint8_t prevHallState = LOW;
	int16_t speed = 0;
	int16_t target_speed = 0;
	static int16_t MAX_CHANGE;
	static int16_t MAX_MOTOR_SPEED;

	Motor(uint8_t forward_pin, uint8_t backward_pin, uint8_t speed_pin, uint8_t hall_pin) 
		: forward_pin(forward_pin), backward_pin(backward_pin), speed_pin(speed_pin), hall_pin(hall_pin) {}

	void setup();
	// Speed should be between -255 and 255
	void setTarget(int16_t speed);
	void updateRotationCount();
	void updateMotorSpeed();
private:
	void turn(int16_t speed);

	friend class MovementController;
};

class MovementController {
public:
	Motor left, right;
	MovementController()
		: left(7, 6, 9, 5), right(8, 12, 10, 4) {}

	void setup();

	void go(int16_t speed, float diff);
	void updateRotationCounts();
	void updateMotorSpeeds();
};
#pragma once
#include <Arduino.h>

class Motor {
public:
	uint8_t forward_pin, backward_pin, speed_pin, hall_pin;
	int hall_pulse_count = 0;
	uint8_t prev_hall_state = LOW;
	int16_t speed = 0;
	int16_t target_speed = 0;

	Motor(uint8_t forward_pin, uint8_t backward_pin, uint8_t speed_pin, uint8_t hall_pin) 
		: forward_pin(forward_pin), backward_pin(backward_pin), speed_pin(speed_pin), hall_pin(hall_pin) {}

	void setup();
	// Speed should be between -255 and 255
	void set_target(int16_t speed);
	void update_rotation_count();
	void update_motor_speed();
	void reset_hall();
private:
	void turn(int16_t speed);

	friend class MotorController;
};
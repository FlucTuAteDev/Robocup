#include "variables.h"

namespace variables
{
	MPU6050 mpu;
	// Accelerometer and gyroscope raw measurments
	int16_t ax, ay, az, gx, gy, gz;

	MotorController mc;
	const int btn = 13;

	float balance_angle = 0;
	bool sampling = false;
	float norm_gyro_x, norm_gyro_z;

	KalmanFilter kalman(0.001, 0.003, 0.5);

	float tilt_angle = 0;

	double p = 34, i = 0, d = 0.62;
	double p_speed = 3.8, i_speed = 0.34, d_speed = 2.5;
	double balance_position = 0;
	int PD_pwm = 0;

	float positions = 0;
	double PI_pwm = 0;
	float speeds_filter;

	int pulseright = 0, pulseleft = 0;
	long cumpulseright = 0, cumpulseleft = 0;

	float pos_constrain = 950;
	float adjust_motor = -500;

	Bluetooth bt;

	// float *MovePhase::position = &positions;
	// float *TurnPhase::position = &positions;
	// float *StopPhase::position = &positions;
	// long *TurnPhase::left_cum_pulse = &cumpulseleft;
	// long *TurnPhase::right_cum_pulse = &cumpulseright;

	// 1. feladat
	//  Phase* phases[] = {
	//  	new WaitPhase(5),
	//  	new MovePhase(10),
	//  	new WaitPhase(5),
	//  	new MovePhase(-10),
	//  	new WaitPhase(5),
	//  	new MovePhase(10),
	//  	new WaitPhase(5),
	//  	new MovePhase(-10),
	//  };
	const int obstacleDistance = 170;
	const int avoidance_radius = 60;
	const int turn_sharpness = 50;
	const float max_diff = 0.3;

	Phase *phases[] = {
		new WaitPhase(1000),
		// new TurnPhase(90),
		// new MovePhase(obstacleDistance - avoidance_radius),

		// // new WaitPhase(1.5),

		// new TurnPhase(turn_sharpness),
		// new TurnPhase(- 2 * turn_sharpness - 180),
		// new TurnPhase(turn_sharpness),

		// new MovePhase(obstacleDistance - avoidance_radius, true),

		// new WaitPhase(100),
		// new MovePhase(30),
	};
	size_t phase_count = sizeof(phases) / sizeof(phases[0]);
	size_t curr_phase_index = 0;
	Phase *curr_phase = phases[0];

	void reset() {
		positions = 0;
		cumpulseleft = 0;
		cumpulseright = 0;
		curr_phase_index = 0;
		curr_phase = phases[0];
		for (size_t i = 0; i < phase_count; i++)
			phases[i]->is_started = false;
	}
}
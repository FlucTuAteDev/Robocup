#pragma once
#include <Arduino.h>
#include <MPU6050.h>
#include <MotorController.h>
#include <KalmanFilter.h>
#include <Bluetooth.h>
#include <Phase.h>

#define DTms 5
#define DTs (DTms / 1000.0)

namespace variables
{
	extern MPU6050 mpu;
	// Accelerometer and gyroscope raw measurments
	extern int16_t ax, ay, az, gx, gy, gz;

	extern MotorController mc;
	extern const int btn;

	extern float balance_angle;
	extern bool sampling;
	extern float norm_gyro_x, norm_gyro_z;

	extern KalmanFilter kalman;

	extern float tilt_angle;

	extern double p, i, d;
	extern double p_speed, i_speed, d_speed;
	extern double balance_position;
	extern int PD_pwm;

	extern float positions;
	extern double PI_pwm;
	extern float speeds_filter;

	extern int pulseright, pulseleft;
	extern long cumpulseright, cumpulseleft;

	extern float pos_constrain;
	extern float adjust_motor;

	extern Bluetooth bt;

	// float *MovePhase::position;
	// float *TurnPhase::position;
	// float *StopPhase::position;
	// long *TurnPhase::left_cum_pulse;
	// long *TurnPhase::right_cum_pulse;

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
	extern const int obstacleDistance;
	extern const int avoidance_radius;
	extern const int turn_sharpness;
	extern const float max_diff;

	extern Phase *phases[];
	extern size_t phase_count;
	extern size_t curr_phase_index;
	extern Phase *curr_phase;

	extern void reset();
}
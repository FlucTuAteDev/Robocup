#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

#include "motor.h"
#include "bluetooth.h"

MPU6050 mpu;

KalmanFilter kalmanX(0.01, 0.03, 0.3);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

// PID
float angle = 0; // Integral
float prev_angle = 0;
float angle_speed = 0; // Proportional
float prev_angle_speed = 0;
float angle_accel = 0; // Derivative

float p_mul = -15;
float d_mul = -200;
float d2_mul = -1000;

MovementController mc;

Bluetooth bluetooth;

unsigned long curr_time;
unsigned long prev_time;

bool stop = true;

void setup()
{
	Serial.begin(9600);
	// Initialize MPU6050
	while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		delay(500);
	}

	mc.setup();

	bluetooth.add_command("p", [](float num){p_mul = num;});
	bluetooth.add_command("d", [](float num){d_mul = num;});
	bluetooth.add_command("d2", [](float num){d2_mul = num;});
	bluetooth.add_command("status", 
		[](float) {
			Serial.println((String)"p_mul = " + p_mul + "; d_mul = " + d_mul + "; d_mul = " + d2_mul + "; hall_count = " + mc.right.hall_pulse_count);
		}
	);
	bluetooth.add_command("start", [](float num){ stop = false; });
	bluetooth.add_command("stop", [](float num){ mc.go(0, 0); stop = true; });


	bluetooth.add_command("go", [](float num) {mc.go(num, 0);});
	// bluetooth.add_command("")

	// Calibrate gyroscope. The calibration must be at rest.
	// mpu.calibrateGyro();

	prev_time = millis();
}

void loop()
{
	bluetooth.poll();
	if (stop) return;

	Vector acc = mpu.readNormalizeAccel();
	Vector gyr = mpu.readNormalizeGyro();

	// Measure dt

	// Calculate roll angle
	prev_angle = angle;
	angle = (atan2(acc.YAxis, acc.ZAxis) * 180.0) / M_PI;
	angle = kalmanX.update(angle, gyr.XAxis);

	mc.updateRotationCounts();
	
	curr_time = millis();
	unsigned long dt = curr_time - prev_time;
	prev_time = curr_time;

	angle_speed = (angle - prev_angle) / dt;

	mc.go(angle * p_mul + angle_speed * d_mul, 0);
}
#include <MsTimer2.h>
#include <PinChangeInterrupt.h>
#include <MPU6050.h>
#include <Wire.h>

#include "KalmanFilter.h"
#include "utils.h"
#include "Bluetooth.h"
#include "MotorController.h"

MPU6050 mpu;
// Accelerometer and gyroscope raw measurments
int16_t ax, ay, az, gx, gy, gz;

MotorController mc;
const int btn = 13;

float balance_angle = 0;
bool sampling = false;
float norm_gyro_x, norm_gyro_z;

KalmanFilter kalman(0.001, 0.003, 0.5);

float angle;
float angle_speed;

double kp = 30, ki = 0.2, kd = 0.5;
double kp_speed = 3.8, ki_speed = 0.34, kd_speed = 2.5;
double balance_position = 0;
int PD_pwm;
float pwm1 = 0, pwm2 = 0;

float speeds_filterold = 0;
float positions = 0;
double PI_pwm;
int piCounter;
float speeds_filter;

int pulseright = 0, pulseleft = 0;
int count_right = 0, count_left = 0;

float Turn_pwm = 0;

Bluetooth bt;

void countpulse();
void angle_calculate();
void PD();
void speedpiout();
void anglePWM();
void DSzhongduan();

void setup()
{
	mc.setup();

	pinMode(btn, INPUT);

	Wire.begin();
	Serial.begin(9600);
	delay(1000);

	mpu.initialize();
	delay(2);

	//	Using timer2 may affect the PWM output of pin 3 and 11
	MsTimer2::set(5, DSzhongduan);
	MsTimer2::start();

	// Start with stopped motors
	mc.stop();

	// External interrupt for calculating wheel speed
	attachPCINT(digitalPinToPCINT(mc.left.hall_pin), []() {count_left++;}, CHANGE);
	attachPCINT(digitalPinToPCINT(mc.right.hall_pin), []() {count_right++;}, CHANGE);

	//region Bluetooth commands
	bt.add_command("motormax", [](float num) { MotorController::max_motor_speed = num; });
	bt.add_command("motorch", [](float num) { MotorController::max_change = num; });
	bt.add_command("i", [](float num) { ki = num; });
	bt.add_command("p", [](float num) { kp = num; });
	bt.add_command("d", [](float num) { kd = num; });
	bt.add_command("angle", [](float num) { balance_angle = num; });
	bt.add_command("start", [](float num) { mc.start(); sampling = false; });
	bt.add_command("stop", [](float num) { mc.stop(); });
	bt.add_command("reset", [](float num) { positions = 0; });
	bt.add_command("status", [](float num) {
		Serial.println((String)"p: " + kp + "; i: " + ki + "; d: " + kd);
		Serial.println((String)"Speed p: " + kp_speed + "; i: " + ki_speed + "; d: " + kd_speed);
		Serial.println((String)"Balance angle: " + balance_angle);
		Serial.println((String)"Angle: " + angle);
		// Serial.println((String)"Positions: " + positions);
		// Serial.println((String)"Speeds filter: " + speeds_filter + "; filterold: " + speeds_filterold);
		Serial.println((String)"PD_pwm: " + PD_pwm);
	});
	bt.add_command("sample", [](float num) {
		mc.stop();
		sampling = true;
		// Start sampling around the angle at which the command is issued
		balance_angle = angle;
	});
	bt.add_command("sample_stop", [](float num) {
		sampling = false;
	});
	//endregion
}

void loop()
{
	if (!digitalRead(btn)) {
		delay(500);
		mc.toggle();
	}
	
	// mc.update_rotation_counts();
	bt.poll();
	// Serial.println(positions);s
	// Serial.println(mc.left.hall_pulse_count);
}

void countpluse()
{
	pulseright += sign(mc.right.speed) * count_right;
	pulseleft += sign(mc.left.speed) * count_left;
	
	count_left = 0; // Clear count quantity
	count_right = 0;
}

void DSzhongduan()
{
	interrupts();

	if (sampling)
		balance_angle = balance_angle * .95 + angle * .05;

	countpluse();
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	angle_calculate();
	PD();
	anglePWM();

	// Run the PI algorithm once every 30 ms (5ms * 6 = 30ms)
	piCounter++;
	if (piCounter >= 6)
	{
		speedpiout();
		piCounter = 0;
	}

	mc.update_motor_speeds();
}

void angle_calculate()
{
	float Angle = -atan2(ay, az) * (180 / PI); // Negative is the direction
	norm_gyro_x = -gx / 131;
	angle = kalman.update(Angle, norm_gyro_x);

	norm_gyro_z = -gz / 131;
}

void PD()
{
	PD_pwm = kp * (angle - balance_angle) + kd * kalman.get_rate();
	// Serial.println(PD_pwm);
}

void speedpiout()
{
	// Negative: adjust in the opposite direction
	float speeds = -(pulseleft + pulseright) * 1.0; // speed  pulse value
	pulseleft = pulseright = 0;
	float pd_tag = speeds_filterold;
	speeds_filterold *= 0.7; // first-order complementary filtering
	speeds_filter = speeds_filterold + speeds * 0.3;
	speeds_filterold = speeds_filter;
	positions += speeds_filter;
	positions = constrain(positions, -4000, 4000);
	PI_pwm = ki_speed * (balance_position - positions) + kp_speed * (balance_position - speeds_filter) + kd_speed * (balance_position - (speeds_filter - pd_tag)); // speed loop control PI
	Serial.println(positions);
}

void anglePWM()
{
	// pwm2 = -PD_pwm - PI_pwm * 1.2 + Turn_pwm;
	// pwm1 = -PD_pwm - PI_pwm * 1.2 - Turn_pwm;

	// pwm1 = constrain(pwm1, -255.0f, 255.0f);
	// pwm2 = constrain(pwm2, -255.0f, 255.0f);
	// Serial.println((String) "" + pwm1 + " " + pwm2);

	// Stop the motors over a certain angle
	if (abs(angle) > 75)
		pwm1 = pwm2 = 0;

	float target = PD_pwm + PI_pwm * 1.2;
	mc.go(target, 0);
	// mc.right.set_target();
	// mc.left.set_target(-pwm2);
}

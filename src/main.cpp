#include <MsTimer2.h>
#include <PinChangeInterrupt.h>
#include <MPU6050.h>
#include <Wire.h>

#include "KalmanFilter.h"
#include "utils.h"
#include "Bluetooth.h"
#include "MotorController.h"
#include "Phase.h"

#define DTms 5
#define DTs (DTms / 1000.0)

MPU6050 mpu;
// Accelerometer and gyroscope raw measurmentsa
int16_t ax, ay, az, gx, gy, gz;

MotorController mc;
const int btn = 13;

float balance_angle = 0;
bool sampling = false;
float norm_gyro_x, norm_gyro_z;

KalmanFilter kalman(0.001, 0.003, 0.5);

float tilt_angle;
float angle_speed;

double kp = 34, ki = 0, kd = 0.62;
double kp_speed = 3.8, ki_speed = 0.34, kd_speed = 2.5;
double kp_turn = 24, kd_turn = 0.08;
double balance_position = 0;
int PD_pwm;

float speeds_filterold = 0;
float positions = 0;
double PI_pwm;
float speeds_filter;

int left = 0, right = 0, turnout = 0;

int pulseright = 0, pulseleft = 0;
long cumpulseright = 0, cumpulseleft = 0;
int16_t turn_speed = 50;
uint8_t turn_time = 20;

float Turn_pwm = 0;
float pos_constrain = 1000;
float adjust_motor = -500;

Bluetooth bt;

float* MovePhase::position = &positions;
float* TurnPhase::position = &positions;
float* StopPhase::position = &positions;
long* TurnPhase::left_cum_pulse = &cumpulseleft;
long* TurnPhase::right_cum_pulse = &cumpulseright;

//1. feladat
// Phase* phases[] = {
// 	new WaitPhase(5),
// 	new MovePhase(10),
// 	new WaitPhase(5),
// 	new MovePhase(-10),
// 	new WaitPhase(5),
// 	new MovePhase(10),
// 	new WaitPhase(5),
// 	new MovePhase(-10),
// };

Phase* phases[] = {
	new WaitPhase(2),
	// new MovePhase(30),
	// new MovePhase(10),

	new TurnPhase(45),
	new TurnPhase(-270),
	new TurnPhase(45),
	// new MovePhase(10),
	new StopPhase(),
	new WaitPhase(100),
	// new MovePhase(30),
};
#define PHASE_COUNT (sizeof(phases) / sizeof(phases[0]))
size_t curr_phase_index = 0;
Phase* curr_phase = phases[0];

void countpulse();
void angle_calculate();
void tilt_control();
void position_control();
void anglePWM();
void DSzhongduan();

void reset() {
	positions = 0;
	cumpulseleft = 0; cumpulseright = 0;
	curr_phase_index = 0;
	curr_phase = phases[0];
	for (size_t i = 0; i < PHASE_COUNT; i++) phases[i]->is_started = false;
}

void setup()
{
	mc.setup();

	pinMode(btn, INPUT);

	// Wire.begin();
	Serial.begin(9600);
	delay(1000);

	mpu.initialize();
	delay(2);

	//	Using timer2 may affect the PWM output of pin 3 and 11
	MsTimer2::set(DTms, DSzhongduan);
	MsTimer2::start();

	// Start with stopped motors
	mc.stop();

	attachPCINT(
		digitalPinToPCINT(mc.left.hall_pin), []()
		{ mc.left.hall_pulse_count++; },
		CHANGE);
	attachPCINT(
		digitalPinToPCINT(mc.right.hall_pin), []()
		{ mc.right.hall_pulse_count++; },
		CHANGE);

	// Bluetooth commands
	{
		bt.add_command("motormax", [](float num) { MotorController::max_motor_speed = num; });
		bt.add_command("motorch", [](float num) { MotorController::max_change = num; });
		bt.add_command("i", [](float num) { ki = num; });
		bt.add_command("p", [](float num) { kp = num; });
		bt.add_command("d", [](float num) { kd = num; });
		bt.add_command("angle", [](float num) { balance_angle = num; });
		bt.add_command("start", [](float num) { mc.start(); sampling = false; });
		bt.add_command("stop", [](float num) { mc.stop(); reset(); });
		bt.add_command("reset", [](float num) { reset(); });
		bt.add_command("status", [](float num) {
			Serial.println((String)"p: " + kp + "; i: " + ki + "; d: " + kd);
			Serial.println((String)"Speed p: " + kp_speed + "; i: " + ki_speed + "; d: " + kd_speed);
			Serial.println((String)"Balance angle: " + balance_angle);
			Serial.println((String)"Pulse left: " + cumpulseleft + "; right: " + cumpulseright);
			Serial.println((String)"Positions: " + positions);
			// Serial.println((String)"Speeds filter: " + speeds_filter + "; filterold: " + speeds_filterold);
			Serial.println((String)"PD_pwm: " + PD_pwm); 
			Serial.println((String)"Current phase index: " + curr_phase_index);
		});
		bt.add_command("sample", [](float num) {
			mc.stop();
			sampling = true;
			// Start sampling around the angle at which the command is issued
			balance_angle = tilt_angle; 
		});
		bt.add_command("sample_stop", [](float num) { sampling = false; });
		bt.add_command("pos", [](float num) { positions += num; });
		bt.add_command("posc", [](float num) { pos_constrain = num; });
		bt.add_command("motora", [](float num) { adjust_motor = num; });
		bt.add_command("pulser", [](float num) { cumpulseright += num; });
		bt.add_command("turns", [](float num) { turn_speed = num; });
		bt.add_command("turnt", [](float num) { turn_time = num; });
	}
}

void loop()
{
	if (!digitalRead(btn))
	{
		delay(500);
		if (mc.stopped)
			reset();
			
		mc.toggle();
	}

	
	if (!curr_phase->is_started && !mc.stopped) {
		Serial.println((String)"Phase started " + curr_phase_index);
		curr_phase->start();
		curr_phase->is_started = true;
	}

	if (curr_phase->is_finished()) {
		Serial.println("Phase finished");

		// Only start next phase if there is one
		if (curr_phase_index + 1 < PHASE_COUNT) {
			curr_phase->is_started = false;
			curr_phase = phases[++curr_phase_index];
		}
	}
	bt.poll();
}

void countpluse()
{
	pulseright += sign(mc.right.speed) * mc.right.hall_pulse_count;
	pulseleft += sign(mc.left.speed) * mc.left.hall_pulse_count;

	cumpulseright += pulseright;
	cumpulseleft += pulseleft;
	mc.reset_halls();
}

void DSzhongduan()
{
	interrupts();

	if (sampling)
		balance_angle = balance_angle * .95 + tilt_angle * .05;

	countpluse();
	angle_calculate();
	tilt_control();
	anglePWM();

	// Run PI every PI_TIME milliseconds
	static uint16_t pi_timer = 0;
	pi_timer += DTms;
	#define PI_TIME 40
	if (pi_timer >= PI_TIME)
	{
		position_control();
		pi_timer = 0;
	}

	static uint16_t turn_timer = 0; // Offset turn timer from PI timer
	turn_timer += DTms;
	#define TURN_TIME 20
	if (turn_timer >= turn_time) {
		// mc.go(turn_speed, 1);
		turn_timer = 0;
	}

	mc.update_motor_speeds();
}

void angle_calculate()
{
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	float Angle = -atan2(ay, az) * (180 / PI); // Negative is the direction
	norm_gyro_x = -gx / 131;
	tilt_angle = kalman.update(Angle, norm_gyro_x);

	norm_gyro_z = -gz / 131;
}

void tilt_control()
{
	PD_pwm = kp * (tilt_angle - balance_angle) + kd * kalman.get_rate();
}

void position_control()
{
	// static float speed;
	// const float max_acceleration = 100;
	// Negative: adjust in the opposite direction
	float speeds = -(pulseleft + pulseright) * 1.0;
	pulseleft = pulseright = 0;
	float pd_tag = speeds_filter;

	speeds_filter = speeds_filter * 0.7 + speeds * 0.3;
	positions += speeds_filter;
	// positions = constrain(positions, -4000, 4000);
	float constrained_pos = constrain(positions, -pos_constrain, pos_constrain);
	PI_pwm = ki_speed * (balance_position - constrained_pos) 
		+ kp_speed * (balance_position - speeds_filter)
		+ kd_speed * (balance_position - (speeds_filter - pd_tag));
}

void anglePWM()
{
	// float right_target = PD_pwm + PI_pwm - Turn_pwm;
	// float left_target = PD_pwm + PI_pwm + Turn_pwm;
	float target = PD_pwm + PI_pwm; // * 1.2;

	// Stop the motors over a certain angle
	if (abs(tilt_angle) > 75)
		target = 0;
		// left_target = right_target = 0;

	// Equalize left and right motor pulses
	// float diff = (cumpulseright - cumpulseleft) / adjust_motor;
	// diff = constrain(diff, -50, 50);

	// mc.go_pwm(left_target + diff, right_target - diff);

	float diff = (cumpulseright - cumpulseleft) / adjust_motor * sign(mc.right.speed);
	
	mc.go(target, constrain(diff, -0.25, 0.25));
}

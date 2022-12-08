#include <MsTimer2.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include "utils.h"

#include "variables.h"
using namespace variables;

void countpulse();
void angle_calculate();
void tilt_control();
void position_control();
void anglePWM();
void DSzhongduan();

void setup()
{
	mc.setup();
	bt.setup_commands();

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
		if (curr_phase_index + 1 < phase_count) {
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
	PD_pwm = p * (tilt_angle - balance_angle) + d * kalman.get_rate();
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
	PI_pwm = i_speed * (balance_position - constrained_pos) 
		+ p_speed * (balance_position - speeds_filter)
		+ d_speed * (balance_position - (speeds_filter - pd_tag));
}

void anglePWM()
{
	// float right_target = PD_pwm + PI_pwm - Turn_pwm;
	// float left_target = PD_pwm + PI_pwm + Turn_pwm;
	float target = PD_pwm + PI_pwm; // * 1.2;

	// Stop the motors over a certain angle
	if (abs(tilt_angle) > 70)
		target = 0;
		// left_target = right_target = 0;

	// Equalize left and right motor pulses
	// float diff = (cumpulseright - cumpulseleft) / adjust_motor;
	// diff = constrain(diff, -50, 50);

	// mc.go_pwm(left_target + diff, right_target - diff);

	float diff = ((cumpulseright - cumpulseleft) / adjust_motor) * sign(mc.right.speed);
	
	mc.go(target, constrain(diff, -max_diff, max_diff));
}

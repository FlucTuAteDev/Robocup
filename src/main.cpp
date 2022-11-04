#include <MsTimer2.h>			//Internal timer2
#include <PinChangeInterrupt.h> //This library file can make all pins on the REV4 board as external interrupts.  Define three-axis acceleration, three-axis gyroscope variables
#include <MPU6050.h>			//MPU6050 Library
#include <Wire.h>				//IIC communication library

#include "KalmanFilter.h"
#include "utils.h"
#include "motor.h"
#include "bluetooth.h"

MPU6050 mpu6050;				// Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz; // Define three-axis acceleration, three-axis gyroscope variables

// TB6612 pins definition
MovementController mc;

const int btn = 13;

///////////////////////angle parameters//////////////////////////////
float angle0 = 0;			  // mechanical balance angle (ideally 0 degrees)
float Gyro_x, Gyro_z; // Angular angular velocity by gyroscope calculation
///////////////////////angle parameter//////////////////////////////

///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.001; // Covariance of gyroscope noise
float Q_gyro = 0.003;  // Covariance of gyroscope drift noise
float R_angle = 0.5;   // Covariance of accelerometer
KalmanFilter kalman(Q_angle, Q_gyro, R_angle);

float angle;
float angle_speed;
//////////////////////Kalman_Filter/////////////////////////

//////////////////////PID parameter///////////////////////////////
double kp = 26, ki = 0.2, kd = 0.9;						// angle loop parameter
double kp_speed = 3.8, ki_speed = 0.34, kd_speed = 2.5; // speed loop parameter
double setp0 = 0;										// angle balance point
int PD_pwm;												// angle output
float pwm1 = 0, pwm2 = 0;

//////////////////interrupt speed count/////////////////////////////
#define PinA_left 5			   // external interrupt
#define PinA_right 4		   // external interrupt
volatile long count_right = 0; // Used to calculate the pulse value calculated by the Hall encoder (the volatile long type is to ensure the value is valid)
volatile long count_left = 0;

int pulseright, pulseleft;
////////////////////////////////PI variable parameter//////////////////////////
float speeds_filterold = 0;
float positions = 0;
double PI_pwm;
int cc;
float speeds_filter;

//////////////////////////////turning PD///////////////////
float Turn_pwm = 0;

int button;

void Kalman_Filter(double angle_m, double gyro_m);
void countpulse();
void angle_calculate(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void PD();
void speedpiout();
void anglePWM();
void DSzhongduan();

Bluetooth bt;

void setup()
{
	// set the motor control pins to OUTPUT
	mc.setup();

	pinMode(btn, INPUT);

	// join I2C bus
	Wire.begin(); // join I2C bus sequence
	Serial.begin(9600);

	mpu6050.initialize(); // initialize MPU6050
	delay(2);

	while (digitalRead(btn))
		;
	// 5ms; use timer2 to set the timer interrupt (note: using timer2 may affects the PWM output of pin3 pin11)
	MsTimer2::set(5, DSzhongduan); // 5ms; execute the function DSzhongduan once
	MsTimer2::start();			   // start interrupt

	// External interrupt for calculating wheel speed
	attachPCINT(digitalPinToPCINT(PinA_left), []() {count_left++;}, CHANGE);	// PinA_left Level change triggers the external interrupt; execute the subfunction Code_left
	attachPCINT(digitalPinToPCINT(PinA_right), []() {count_right++;}, CHANGE); // PinA_right Level change triggers the external interrupt; execute the subfunction Code_right
	bt.add_command("motormax", [](float num) { Motor::MAX_MOTOR_SPEED = num; });
	bt.add_command("motorch", [](float num) { Motor::MAX_CHANGE = num; });
	bt.add_command("i", [](float num) { ki = num; });
	bt.add_command("p", [](float num) { kp = num; });
	bt.add_command("d", [](float num) { kd = num; });
	bt.add_command("angle", [](float num) { angle0 = num; });
}

void loop()
{
	// Serial.println(angle);
}

////////////////////pulse count///////////////////////
void countpluse()
{
	pulseright += sign(pwm2) * count_right;
	pulseleft += sign(pwm1) * count_left;
	
	count_left = 0; // Clear count quantity
	count_right = 0;
}

/////////////////////////////////interrupt ////////////////////////////
void DSzhongduan()
{
	sei();																			// allow overall interrupt
	countpluse();																	// pulse plus subfunction
	mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);								// IIC to get MPU6050 six-axis data  ax ay az gx gy gz
	angle_calculate(ax, ay, az, gx, gy, gz); // get angle and Kalmam filtering
	PD();																			// angle loop PD control
	anglePWM();

	cc++;
	if (cc >= 8) // 5*8=40，enter PI algorithm of speed per 40ms
	{
		speedpiout();
		cc = 0; // Clear
	}

	mc.updateMotorSpeeds();
}
///////////////////////////////////////////////////////////

/////////////////////////////tilt calculation///////////////////////
void angle_calculate(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
	float Angle = -atan2(ay, az) * (180 / PI); // Radial rotation angle calculation formula ; negative sign is direction processing
	Gyro_x = -gx / 131;						   // The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
	angle = kalman.update(Angle, Gyro_x);			   // Kalman Filter

	// rotating angle Z-axis parameter
	Gyro_z = -gz / 131; // angle speed of Z-axis
	// accelz = az / 1604;
}

//////////////////angle PD////////////////////
void PD()
{
	PD_pwm = kp * (angle + angle0) + kd * kalman.getRate() + angle * ki; // PD angle loop control
	// Serial.println(PD_pwm);
}

//////////////////speed PI////////////////////
void speedpiout()
{
	float speeds = (pulseleft + pulseright) * 1.0; // speed  pulse value
	pulseright = pulseleft = 0;					   // clear
	float pd_tag = speeds_filterold;
	speeds_filterold *= 0.7; // first-order complementary filtering
	speeds_filter = speeds_filterold + speeds * 0.3;
	speeds_filterold = speeds_filter;
	positions += speeds_filter;
	positions = constrain(positions, -4000, 4000);																				  // Anti-integral saturation
	PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter) + kd_speed * (setp0 - (speeds_filter - pd_tag)); // speed loop control PI
}
//////////////////speed PI////////////////////

////////////////////////////PWM end value/////////////////////////////
void anglePWM()
{
	pwm2 = -PD_pwm - PI_pwm * 1.2 + Turn_pwm; // assign the end value of PWM to motor
	pwm1 = -PD_pwm - PI_pwm * 1.2 - Turn_pwm;

	pwm1 = constrain(pwm1, -255.0f, 255.0f);
	pwm2 = constrain(pwm2, -255.0f, 255.0f);
	// Serial.println((String) "" + pwm1 + " " + pwm2);

	if (abs(angle) > 45) // if tilt angle is greater than 45°，motor will stop
		pwm1 = pwm2 = 0;

	mc.right.setTarget(-pwm1);
	mc.left.setTarget(-pwm2);
}

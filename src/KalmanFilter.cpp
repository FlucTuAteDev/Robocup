#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double angle, double bias, double measure)
{
    Q_angle = angle;
    Q_bias = bias;
    R_measure = measure;

    K_angle = 0;
    K_bias = 0;

    P[0][0] = 1;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 1;

    kt = (double)micros();
}

double KalmanFilter::update(double newValue, double newRate)
{
	dt = (double)(micros() - kt) / 1000000;

	K_angle += (newRate - K_bias) * dt;
	angle_err = newValue - K_angle;

	P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt;
	P[0][1] -= P[1][1] * dt;
	P[1][0] -= P[1][1] * dt;
	P[1][1] += Q_bias * dt;

	S = R_measure + P[0][0];
	
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

	K_bias += K[1] * angle_err;
	K_rate = newRate - K_bias;
	K_angle += K[0] * angle_err;

	kt = (double)micros();

	return K_angle;
}
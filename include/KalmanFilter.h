#pragma once

class KalmanFilter
{
    public:

	KalmanFilter(double angle = 0.001, double bias = 0.003, double measure = 0.03);
	double update(double new_value, double new_rate);
	double get_rate() { return K_rate; }

    private:

	double S, angle_err;
	double Q_angle, Q_bias, R_measure;
	double K_angle, K_bias, K_rate;
	double P[2][2], K[2];
	double dt, kt;
};
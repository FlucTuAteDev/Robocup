#pragma once
#include <Arduino.h>

class Phase
{
public:
	bool is_started = false;

	virtual void start() = 0;
	virtual bool is_finished() = 0;
};

class WaitPhase : public Phase {
private:
	long start_time;
	size_t delay;
public:
	WaitPhase(size_t delay);
	void start() override;
	bool is_finished() override;
};

class MovePhase : public Phase {
private:
	int distance;
public:
	static float* position;
	
	MovePhase(int distance);
	void start() override;
	bool is_finished() override;
};

class TurnPhase : public Phase {
private:
	// float forward_speed;
	float angle;
public:
	static float* position;
	static long* right_cum_pulse, *left_cum_pulse;

	// TurnPhase(float forward_speed, long angle);
	TurnPhase(float angle);

	void start() override;
	bool is_finished() override;
};

class StopPhase : public Phase {
public:	
	StopPhase();
	void start() override;
	bool is_finished() override;
	static float* position;
};
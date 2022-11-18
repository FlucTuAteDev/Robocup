#include "Phase.h"

WaitPhase::WaitPhase(size_t delay) : delay(delay) {}
void WaitPhase::start()
{
	start_time = millis();
}
bool WaitPhase::is_finished()
{
	if (!is_started)
		return false;

	if (millis() - start_time >= delay)
	{
		return true;
	}

	return false;
}

MovePhase::MovePhase(int distance) : distance(distance) {}
void MovePhase::start()
{
	*position += distance;
}
bool MovePhase::is_finished()
{
	return abs(*position) < abs(distance) / 30;
}

TurnPhase::TurnPhase(float forward_speed, long angle) : forward_speed(forward_speed), angle(angle) {}
void TurnPhase::start()
{
	*position += forward_speed;
	*right_cum_pulse += angle;
}
bool TurnPhase::is_finished()
{
	return abs(*left_cum_pulse - *right_cum_pulse) <= 200;
}
#include "Phase.h"

WaitPhase::WaitPhase(size_t sec) : delay(sec * 1000) {}
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

#define BALANCE_POS 100
#define PPR 66
#define ROBOT_RADIUS 11.6

MovePhase::MovePhase(int cm) : distance(cm) {}
void MovePhase::start()
{
	*position += distance * PPR;
}
bool MovePhase::is_finished()
{
	return abs(*position - BALANCE_POS) < abs(distance * PPR / 15);
}

TurnPhase::TurnPhase(float angle) : angle(angle) {}
void TurnPhase::start()
{
	float outer_circumference = angle / 360 * (8 * ROBOT_RADIUS * PI * PPR) ;
	*position += outer_circumference;
	*right_cum_pulse += outer_circumference;
}
bool TurnPhase::is_finished()
{
	return abs(*left_cum_pulse - *right_cum_pulse) <= 200;
}